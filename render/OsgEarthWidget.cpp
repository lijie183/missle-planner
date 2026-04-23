#include "render/OsgEarthWidget.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <random>
#include <string>

#include <QPaintEngine>
#include <QResizeEvent>
#include <QShowEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QFileInfo>

#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/Quat>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osg/Vec3d>
#include <osg/Image>

#include <osgDB/ReadFile>

#include <osgGA/GUIEventAdapter>
#include <osgGA/TrackballManipulator>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>

#ifdef _WIN32
#include <osgViewer/api/Win32/GraphicsWindowWin32>
#endif

#include <osgEarth/GeoData>
#include <osgEarth/GeoTransform>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Profile>
#include <osgEarth/SpatialReference>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Viewpoint>
#include <osgEarth/XYZ>

#include "core/MissionTypes.h"

namespace {

constexpr double kMetersPerLatDegree = 111320.0;
constexpr double kFallbackEarthRadiusMeters = 6378137.0;

double toRadians(double degrees) {
    return degrees * 0.017453292519943295;
}

double approximateDistanceMeters(const osgEarth::GeoPoint& a, const osgEarth::GeoPoint& b) {
    const double meanLatRad = toRadians((a.y() + b.y()) * 0.5);
    const double metersPerLonDegree = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));

    const double dx = (b.x() - a.x()) * metersPerLonDegree;
    const double dy = (b.y() - a.y()) * kMetersPerLatDegree;
    const double dz = b.z() - a.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

osg::Vec3d geodeticToWorld(double lonDeg, double latDeg, double radiusMeters) {
    const double lonRad = toRadians(lonDeg);
    const double latRad = toRadians(latDeg);
    const double cosLat = std::cos(latRad);
    return osg::Vec3d(
        radiusMeters * cosLat * std::cos(lonRad),
        radiusMeters * cosLat * std::sin(lonRad),
        radiusMeters * std::sin(latRad));
}

double wrapDeltaLon(double delta) {
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
}

int scaledPixels(int logicalPixels, double dpr) {
    const double scaled = static_cast<double>(std::max(1, logicalPixels)) * std::max(1.0, dpr);
    return std::max(1, static_cast<int>(std::lround(scaled)));
}

osg::ref_ptr<osg::Node> buildOnlineRealEarthNode(osg::ref_ptr<osgEarth::MapNode>& outMapNode) {
    osg::ref_ptr<osgEarth::Map> map = new osgEarth::Map;

    osgEarth::XYZImageLayer::Options imageryOptions;
    imageryOptions.url() = osgEarth::URI(
        "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}");
    imageryOptions.format() = std::string("jpg");
    imageryOptions.minLevel() = 0u;
    imageryOptions.maxLevel() = 18u;

    osg::ref_ptr<osgEarth::XYZImageLayer> imageryLayer = new osgEarth::XYZImageLayer(imageryOptions);
    imageryLayer->setName("SatelliteImagery");
    map->addLayer(imageryLayer.get());

    osgEarth::XYZElevationLayer::Options elevationOptions;
    elevationOptions.url() = osgEarth::URI(
        "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png");
    elevationOptions.format() = std::string("png");
    elevationOptions.minLevel() = 0u;
    elevationOptions.maxLevel() = 13u;
    elevationOptions.elevationEncoding() = std::string("terrarium");
    elevationOptions.stitchEdges() = true;

    osg::ref_ptr<osgEarth::XYZElevationLayer> elevationLayer = new osgEarth::XYZElevationLayer(elevationOptions);
    elevationLayer->setName("GlobalTerrain");
    map->addLayer(elevationLayer.get());

    outMapNode = new osgEarth::MapNode(map.get());
    if (!outMapNode.valid() || outMapNode->getTerrainEngine() == nullptr) {
        outMapNode = nullptr;
        return {};
    }
    return outMapNode;
}

double gaussianBlob(double lonDeg, double latDeg, double centerLon, double centerLat, double sigmaLon, double sigmaLat) {
    const double dx = wrapDeltaLon(lonDeg - centerLon);
    const double dy = latDeg - centerLat;
    const double sx2 = std::max(1.0, sigmaLon * sigmaLon);
    const double sy2 = std::max(1.0, sigmaLat * sigmaLat);
    return std::exp(-(dx * dx / (2.0 * sx2) + dy * dy / (2.0 * sy2)));
}

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

osg::Vec4 mixColor(const osg::Vec4& a, const osg::Vec4& b, double t) {
    const double safeT = clamp01(t);
    return osg::Vec4(
        static_cast<float>(a.r() + (b.r() - a.r()) * safeT),
        static_cast<float>(a.g() + (b.g() - a.g()) * safeT),
        static_cast<float>(a.b() + (b.b() - a.b()) * safeT),
        static_cast<float>(a.a() + (b.a() - a.a()) * safeT));
}

osg::Matrixd makeUpAlignedMatrix(const osg::Vec3d& worldPos) {
    osg::Vec3d up = worldPos;
    if (up.length2() < 1e-6) {
        up = osg::Vec3d(0.0, 0.0, 1.0);
    } else {
        up.normalize();
    }

    osg::Quat rotation;
    rotation.makeRotate(osg::Vec3d(0.0, 0.0, 1.0), up);
    return osg::Matrix::rotate(rotation) * osg::Matrix::translate(worldPos);
}

osg::ref_ptr<osg::Geode> buildRingGeode(double radiusMeters, double zOffsetMeters, const osg::Vec4& color) {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    constexpr int segments = 72;
    const double fullTurn = 2.0 * 3.14159265358979323846;
    vertices->reserve(segments);

    for (int i = 0; i < segments; ++i) {
        const double angle = fullTurn * static_cast<double>(i) / static_cast<double>(segments);
        vertices->push_back(osg::Vec3(
            static_cast<float>(std::cos(angle) * radiusMeters),
            static_cast<float>(std::sin(angle) * radiusMeters),
            static_cast<float>(zOffsetMeters)));
    }

    osg::ref_ptr<osg::Geometry> ring = new osg::Geometry;
    ring->setVertexArray(vertices.get());
    ring->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(vertices->size())));

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(color);
    ring->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

    osg::StateSet* stateSet = ring->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth(2.0f);
    stateSet->setAttributeAndModes(width.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(ring.get());
    return geode;
}

osg::ref_ptr<osg::MatrixTransform> buildThreatVisualNode(const mission::ThreatZone& threat) {
    const double height = std::max(80.0, threat.maxAltitudeMeters - threat.minAltitudeMeters);
    const double centerAlt = threat.minAltitudeMeters + height * 0.5;
    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) {
        return nullptr;
    }

    osgEarth::GeoPoint geoPoint(
        wgs84,
        threat.longitudeDeg,
        threat.latitudeDeg,
        centerAlt,
        osgEarth::ALTMODE_ABSOLUTE);

    osg::Vec3d world;
    geoPoint.toWorld(world);

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
    xform->setMatrix(makeUpAlignedMatrix(world));

    const double threatScale = clamp01(0.2 + threat.radiusMeters / 45000.0 + height / 12000.0);
    const osg::Vec4 hotCore = mixColor(osg::Vec4(1.0f, 0.90f, 0.20f, 0.78f), osg::Vec4(1.0f, 0.18f, 0.08f, 0.92f), threatScale);
    const osg::Vec4 coreColor(hotCore.r(), hotCore.g(), hotCore.b(), 0.42f);
    const osg::Vec4 shellColor(1.0f, 0.42f + static_cast<float>(0.18 * threatScale), 0.08f, 0.24f);
    const osg::Vec4 hazeColor(1.0f, 0.22f, 0.10f, 0.12f);
    const osg::Vec4 ringColor(1.0f, 0.94f, 0.28f, 0.45f);
    const osg::Vec4 pillarColor(1.0f, 0.96f, 0.34f, 0.72f);

    auto addCylinder = [&](double radiusMeters, double cylinderHeight, const osg::Vec4& color, double zOffsetMeters) {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(
            new osg::Cylinder(osg::Vec3(0.0f, 0.0f, static_cast<float>(zOffsetMeters)),
                              static_cast<float>(radiusMeters),
                              static_cast<float>(cylinderHeight)));
        drawable->setColor(color);
        geode->addDrawable(drawable.get());

        osg::StateSet* stateSet = geode->getOrCreateStateSet();
        stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
        stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

        xform->addChild(geode.get());
    };

    addCylinder(threat.radiusMeters * 1.12, height * 0.92, hazeColor, 0.0);
    addCylinder(threat.radiusMeters * 0.86, height * 0.98, shellColor, 0.0);
    addCylinder(threat.radiusMeters * 0.52, height * 1.04, coreColor, 0.0);
    addCylinder(std::max(160.0, threat.radiusMeters * 0.11), height * 1.08, pillarColor, 0.0);

    xform->addChild(buildRingGeode(threat.radiusMeters * 1.12, static_cast<double>(height) * 0.5, ringColor).get());
    xform->addChild(buildRingGeode(threat.radiusMeters * 1.12, -static_cast<double>(height) * 0.5, ringColor).get());

    return xform;
}

osg::ref_ptr<osg::Node> buildVerticalMarkerNode(const osgEarth::GeoPoint& point, const osg::Vec4& color, double radiusMeters) {
    osg::Vec3d world;
    point.toWorld(world);

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
    xform->setMatrix(makeUpAlignedMatrix(world));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> pillar = new osg::ShapeDrawable(
        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.0f),
                          static_cast<float>(radiusMeters),
                          static_cast<float>(std::max(400.0, point.z() * 1.02))));
    pillar->setColor(color);
    geode->addDrawable(pillar.get());

    osg::ref_ptr<osg::ShapeDrawable> cap = new osg::ShapeDrawable(
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, static_cast<float>(std::max(400.0, point.z() * 1.02) * 0.5)),
                        static_cast<float>(radiusMeters * 1.35)));
    cap->setColor(osg::Vec4(color.r(), color.g(), color.b(), std::min(0.92f, color.a() + 0.12f)));
    geode->addDrawable(cap.get());

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    xform->addChild(geode.get());
    return xform;
}

double headingDegrees(const osgEarth::GeoPoint& from, const osgEarth::GeoPoint& to) {
    const double meanLatRad = toRadians((from.y() + to.y()) * 0.5);
    const double dx = (to.x() - from.x()) * kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double dy = (to.y() - from.y()) * kMetersPerLatDegree;

    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        return 0.0;
    }

    double heading = std::atan2(dx, dy) * 57.29577951308232;
    if (heading < 0.0) {
        heading += 360.0;
    }
    return heading;
}

double pitchDegrees(const osgEarth::GeoPoint& from, const osgEarth::GeoPoint& to) {
    const double meanLatRad = toRadians((from.y() + to.y()) * 0.5);
    const double dx = (to.x() - from.x()) * kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double dy = (to.y() - from.y()) * kMetersPerLatDegree;
    const double horizontal = std::sqrt(dx * dx + dy * dy);
    const double dz = to.z() - from.z();
    if (horizontal < 1e-6 && std::abs(dz) < 1e-6) {
        return 0.0;
    }
    return std::atan2(dz, std::max(1e-3, horizontal)) * 57.29577951308232;
}

osg::ref_ptr<osg::Image> tryLoadEarthTextureImage() {
    const std::array<std::string, 6> imageCandidates = {
        "resources/earth_day.jpg",
        "data/earth_day.jpg",
        "earth_day.jpg",
        "Images/land_shallow_topo_2048.jpg",
        "land_shallow_topo_2048.jpg",
        "world.topo.bathy.200401.jpg"};

    for (const auto& fileName : imageCandidates) {
        if (!QFileInfo::exists(QString::fromStdString(fileName))) {
            continue;
        }
        osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(fileName);
        if (image.valid()) {
            return image;
        }
    }

    return {};
}

osg::ref_ptr<osg::Image> buildProceduralEarthTexture() {
    constexpr int width = 2048;
    constexpr int height = 1024;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);

    for (int y = 0; y < height; ++y) {
        const double v = static_cast<double>(y) / static_cast<double>(height - 1);
        const double lat = 90.0 - 180.0 * v;
        for (int x = 0; x < width; ++x) {
            const double u = static_cast<double>(x) / static_cast<double>(width - 1);
            const double lon = -180.0 + 360.0 * u;

            double continent = 0.0;
            continent += 1.35 * gaussianBlob(lon, lat, 80.0, 47.0, 34.0, 18.0);
            continent += 1.05 * gaussianBlob(lon, lat, 104.0, 30.0, 22.0, 13.0);
            continent += 0.92 * gaussianBlob(lon, lat, 22.0, 8.0, 24.0, 20.0);
            continent += 1.02 * gaussianBlob(lon, lat, -100.0, 46.0, 29.0, 19.0);
            continent += 0.95 * gaussianBlob(lon, lat, -62.0, -17.0, 19.0, 16.0);
            continent += 0.84 * gaussianBlob(lon, lat, 135.0, -24.0, 16.0, 9.0);
            continent += 0.72 * gaussianBlob(lon, lat, -42.0, 72.0, 12.0, 7.0);

            const double terrainNoise =
                0.20 * std::sin(toRadians(lon * 3.0)) * std::cos(toRadians(lat * 4.0)) +
                0.14 * std::sin(toRadians((lon + lat) * 6.0)) +
                0.08 * std::cos(toRadians(lon * 11.0 - lat * 2.0));

            const double landMask = continent + terrainNoise;
            const bool isLand = landMask > 0.66;
            const double polar = std::clamp((std::abs(lat) - 58.0) / 30.0, 0.0, 1.0);

            double r = 0.0;
            double g = 0.0;
            double b = 0.0;

            if (isLand) {
                const double moist = std::clamp(0.5 + 0.5 * std::sin(toRadians(lat * 2.4 + lon * 0.3)), 0.0, 1.0);
                const double relief = std::clamp(0.5 + landMask * 0.25, 0.0, 1.0);
                r = 58.0 + 78.0 * relief + 24.0 * (1.0 - moist);
                g = 88.0 + 102.0 * moist;
                b = 42.0 + 36.0 * relief;
            } else {
                const double shallow = std::clamp(1.1 - (0.66 - landMask) * 2.0, 0.0, 1.0);
                r = 8.0 + 18.0 * shallow;
                g = 28.0 + 46.0 * shallow;
                b = 70.0 + 95.0 * shallow;
            }

            if (polar > 0.02) {
                const double iceBlend = std::clamp(polar * 0.95, 0.0, 0.95);
                r = r * (1.0 - iceBlend) + 224.0 * iceBlend;
                g = g * (1.0 - iceBlend) + 236.0 * iceBlend;
                b = b * (1.0 - iceBlend) + 244.0 * iceBlend;
            }

            auto* pixel = image->data(x, y);
            pixel[0] = static_cast<std::uint8_t>(std::clamp(r, 0.0, 255.0));
            pixel[1] = static_cast<std::uint8_t>(std::clamp(g, 0.0, 255.0));
            pixel[2] = static_cast<std::uint8_t>(std::clamp(b, 0.0, 255.0));
        }
    }

    image->setInternalTextureFormat(GL_RGB);
    return image;
}

osg::ref_ptr<osg::Geode> buildStarFieldGeode() {
    osg::ref_ptr<osg::Geode> starsGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> stars = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;

    std::mt19937 rng(1337u);
    std::uniform_real_distribution<double> lonDist(-180.0, 180.0);
    std::uniform_real_distribution<double> latDist(-90.0, 90.0);
    std::uniform_real_distribution<double> lumDist(0.35, 1.0);

    constexpr int starCount = 2800;
    constexpr double starRadius = 75000000.0;

    vertices->reserve(starCount);
    colors->reserve(starCount);

    for (int i = 0; i < starCount; ++i) {
        const double lon = lonDist(rng);
        const double lat = latDist(rng);
        const double lum = lumDist(rng);
        vertices->push_back(geodeticToWorld(lon, lat, starRadius));
        colors->push_back(osg::Vec4(0.6f + 0.4f * lum, 0.7f + 0.3f * lum, 1.0f, 0.9f));
    }

    stars->setVertexArray(vertices.get());
    stars->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
    stars->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertices->size())));

    osg::StateSet* stateSet = stars->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateSet->setAttribute(new osg::Point(2.0f));

    starsGeode->addDrawable(stars.get());
    return starsGeode;
}

osg::ref_ptr<osg::Node> tryLoadEarthFile(std::string* outLoadedPath) {
    const std::array<std::string, 15> candidates = {
        "highres_global.earth", "default.earth", "world.earth",
        "data/highres_global.earth", "data/default.earth", "data/world.earth",
        "data/earth/highres_global.earth", "data/earth/default.earth", "data/earth/world.earth",
        "resources/highres_global.earth", "resources/default.earth", "resources/world.earth",
        "resources/earth/highres_global.earth", "resources/earth/default.earth", "resources/earth/world.earth"};

    for (const auto& fileName : candidates) {
        if (!QFileInfo::exists(QString::fromStdString(fileName))) {
            continue;
        }
        osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(fileName);
        if (node.valid()) {
            if (outLoadedPath != nullptr) {
                *outLoadedPath = fileName;
            }
            return node;
        }
    }

    return {};
}

osg::ref_ptr<osg::Node> buildMarkerNode(const osgEarth::GeoPoint& point, const osg::Vec4& color, double radiusMeters) {
    osg::Vec3d world;
    point.toWorld(world);

    osg::ref_ptr<osg::MatrixTransform> worldTransform = new osg::MatrixTransform;
    worldTransform->setMatrix(osg::Matrix::translate(world));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> marker = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), radiusMeters));
    marker->setColor(color);
    geode->addDrawable(marker.get());

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    worldTransform->addChild(geode.get());
    return worldTransform;
}

osg::ref_ptr<osg::Group> buildMissileModel(
    const osg::Vec4& bodyColor,
    osg::ref_ptr<osg::MatrixTransform>& outExhaustNode,
    osg::ref_ptr<osg::ShapeDrawable>& outExhaustDrawable) {
    osg::ref_ptr<osg::Group> root = new osg::Group;

    osg::ref_ptr<osg::Geode> bodyGeode = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> fuselage = new osg::ShapeDrawable(
        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, -600.0f), 620.0f, 5600.0f));
    fuselage->setColor(osg::Vec4(
        std::min(1.0f, bodyColor.r() * 0.7f + 0.25f),
        std::min(1.0f, bodyColor.g() * 0.7f + 0.25f),
        std::min(1.0f, bodyColor.b() * 0.7f + 0.25f),
        0.98f));
    bodyGeode->addDrawable(fuselage.get());

    osg::ref_ptr<osg::ShapeDrawable> nose = new osg::ShapeDrawable(
        new osg::Cone(osg::Vec3(0.0f, 0.0f, 2600.0f), 620.0f, 1900.0f));
    nose->setColor(osg::Vec4(0.94f, 0.97f, 1.0f, 1.0f));
    bodyGeode->addDrawable(nose.get());

    osg::ref_ptr<osg::ShapeDrawable> ring = new osg::ShapeDrawable(
        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 1200.0f), 680.0f, 260.0f));
    ring->setColor(osg::Vec4(0.16f, 0.20f, 0.26f, 1.0f));
    bodyGeode->addDrawable(ring.get());

    osg::StateSet* bodyState = bodyGeode->getOrCreateStateSet();
    bodyState->setMode(GL_BLEND, osg::StateAttribute::ON);
    bodyState->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    bodyState->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    root->addChild(bodyGeode.get());

    osg::ref_ptr<osg::Geode> finGeode = new osg::Geode;
    auto addFin = [&](double yawRad) {
        const float c = static_cast<float>(std::cos(yawRad));
        const float s = static_cast<float>(std::sin(yawRad));
        osg::ref_ptr<osg::Vec3Array> finVertices = new osg::Vec3Array;
        finVertices->push_back(osg::Vec3(720.0f * c, 720.0f * s, -1500.0f));
        finVertices->push_back(osg::Vec3(2050.0f * c, 2050.0f * s, -1600.0f));
        finVertices->push_back(osg::Vec3(1250.0f * c, 1250.0f * s, -2600.0f));

        osg::ref_ptr<osg::Geometry> fin = new osg::Geometry;
        fin->setVertexArray(finVertices.get());
        fin->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, 3));

        osg::ref_ptr<osg::Vec4Array> finColors = new osg::Vec4Array;
        finColors->push_back(osg::Vec4(0.16f, 0.20f, 0.28f, 0.95f));
        fin->setColorArray(finColors.get(), osg::Array::BIND_OVERALL);
        finGeode->addDrawable(fin.get());
    };

    addFin(0.0);
    addFin(3.14159265358979323846 * 0.5);
    addFin(3.14159265358979323846);
    addFin(3.14159265358979323846 * 1.5);
    root->addChild(finGeode.get());

    outExhaustNode = new osg::MatrixTransform;
    outExhaustNode->setMatrix(osg::Matrix::translate(0.0, 0.0, -3500.0));

    osg::ref_ptr<osg::Geode> exhaustGeode = new osg::Geode;
    outExhaustDrawable = new osg::ShapeDrawable(
        new osg::Cone(osg::Vec3(0.0f, 0.0f, -450.0f), 460.0f, 2200.0f));
    outExhaustDrawable->setColor(osg::Vec4(1.0f, 0.66f, 0.18f, 0.86f));
    exhaustGeode->addDrawable(outExhaustDrawable.get());

    osg::StateSet* exhaustState = exhaustGeode->getOrCreateStateSet();
    exhaustState->setMode(GL_BLEND, osg::StateAttribute::ON);
    exhaustState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    exhaustState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    exhaustState->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    outExhaustNode->addChild(exhaustGeode.get());

    root->addChild(outExhaustNode.get());
    return root;
}

}  // namespace

OsgEarthWidget::OsgEarthWidget(QWidget* parent)
    : QWidget(parent) {
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setAutoFillBackground(false);
    setContextMenuPolicy(Qt::NoContextMenu);

    auto* frameTimer = new QTimer(this);
    frameTimer->setTimerType(Qt::PreciseTimer);
    connect(frameTimer, &QTimer::timeout, this, [this]() {
        if (m_viewer.valid() && isVisible()) {
            m_viewer->frame();
        }
    });
    frameTimer->start(16);
}

OsgEarthWidget::~OsgEarthWidget() {
    if (m_viewer.valid()) {
        m_viewer->setDone(true);
    }
}

void OsgEarthWidget::zoomBySteps(int steps) {
    if (!m_graphicsWindow.valid() || steps == 0) {
        return;
    }

    const osgGA::GUIEventAdapter::ScrollingMotion motion =
        steps > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN;
    for (int i = 0; i < std::abs(steps); ++i) {
        m_graphicsWindow->getEventQueue()->mouseScroll(motion);
    }
}

void OsgEarthWidget::setGlobeMode(GlobeMode mode) {
    m_globeMode = mode;
    if (m_initialized) {
        rebuildFallbackGlobe();
        applyGlobeMode();
        updateCameraManipulator();
    }
}

OsgEarthWidget::GlobeMode OsgEarthWidget::globeMode() const {
    return m_globeMode;
}

bool OsgEarthWidget::hasRealEarthDataset() {
    ensureSceneCreated();
    return m_hasRealEarthDataset;
}

QString OsgEarthWidget::realEarthStatusText() const {
    if (m_globeMode == GlobeMode::Presentation) {
        return QStringLiteral("演示简化模型");
    }

    if (!m_sceneCreationAttempted && !m_root.valid()) {
        return QStringLiteral("待检测（运行后加载）");
    }

    if (m_hasRealEarthDataset) {
        if (m_realEarthFromLocalFile && !m_realEarthSourcePath.isEmpty()) {
            return QStringLiteral("离线卫星数据：%1").arg(m_realEarthSourcePath);
        }
        return QStringLiteral("在线卫星影像+在线地形");
    }

    return QStringLiteral("程序化真实纹理（未检测到可用地形引擎）");
}

void OsgEarthWidget::setStartPoint(const osgEarth::GeoPoint& point) {
    setMissileStartPoint(0, point);
}

void OsgEarthWidget::setTargetPoint(const osgEarth::GeoPoint& point) {
    setMissileTargetPoint(0, point);
}

void OsgEarthWidget::setThreatZones(const std::vector<mission::ThreatZone>& threats) {
    m_threats = threats;
    if (m_root.valid()) {
        rebuildThreatGeometry();
    }
}

void OsgEarthWidget::setPlannedRoute(const std::vector<osgEarth::GeoPoint>& route) {
    setMissileRoute(0, route);
}

void OsgEarthWidget::setMissilePosition(const osgEarth::GeoPoint& position) {
    setMissilePosition(0, position);
}

void OsgEarthWidget::setFollowMissile(bool enabled) {
    setFollowMissile(0, enabled);
}

void OsgEarthWidget::focusOnPoint(const osgEarth::GeoPoint& point, double rangeMeters) {
    if (!point.isValid()) {
        return;
    }

    ensureSceneCreated();

    const double safeRange = std::clamp(rangeMeters, 30000.0, 1200000.0);

    if (m_globeMode == GlobeMode::Realistic && m_hasRealEarthDataset && m_earthManipulator.valid()) {
        osgEarth::Viewpoint vp(
            "mission-focus",
            point.x(), point.y(), point.z(),
            20.0, -28.0, safeRange);
        m_earthManipulator->setViewpoint(vp, 0.25);
        return;
    }

    if (!m_trackballManipulator.valid()) {
        return;
    }

    osg::Vec3d center;
    point.toWorld(center);

    osg::Vec3d up = center;
    if (up.length2() < 1e-6) {
        up = osg::Vec3d(0.0, 0.0, 1.0);
    } else {
        up.normalize();
    }

    osg::Vec3d side = up ^ osg::Vec3d(0.0, 0.0, 1.0);
    if (side.length2() < 1e-6) {
        side = up ^ osg::Vec3d(0.0, 1.0, 0.0);
    }
    side.normalize();

    const osg::Vec3d eye = center + up * safeRange + side * (safeRange * 0.15);
    m_trackballManipulator->setTransformation(eye, center, up);
}

void OsgEarthWidget::focusOnRoute(const std::vector<osgEarth::GeoPoint>& route) {
    if (route.empty()) {
        return;
    }

    const osgEarth::GeoPoint& start = route.front();
    const osgEarth::GeoPoint& goal = route.back();
    const osgEarth::GeoPoint& center = route[route.size() / 2];
    const double baselineMeters = approximateDistanceMeters(start, goal);
    const double altitudeSpan = std::abs(goal.z() - start.z());
    const double rangeMeters = std::clamp(baselineMeters * 1.45 + altitudeSpan * 6.0 + 45000.0, 90000.0, 1500000.0);

    ensureSceneCreated();
    if (m_globeMode == GlobeMode::Realistic && m_hasRealEarthDataset && m_earthManipulator.valid()) {
        osgEarth::Viewpoint vp(
            "mission-route",
            center.x(), center.y(), center.z(),
            headingDegrees(start, goal),
            -24.0, rangeMeters);
        m_earthManipulator->setViewpoint(vp, 0.25);
        return;
    }

    focusOnPoint(center, rangeMeters);
}

void OsgEarthWidget::showImpactEffect(const osgEarth::GeoPoint& point) {
    showMissileImpact(0, point);
}

void OsgEarthWidget::clearImpactEffect() {
    clearMissileImpact(0);
}

void OsgEarthWidget::clearMissile() {
    clearMissile(0);
}

void OsgEarthWidget::resetMissionGraphics() {
    m_threats.clear();
    m_activeMissileIndex = -1;
    for (auto& mv : m_missileVisuals) {
        mv.route.clear();
        mv.trail.clear();
        mv.hasStart = false;
        mv.hasTarget = false;
        mv.hasMissile = false;
        mv.hasPreviousPoint = false;
        mv.followEnabled = false;
        mv.followTickCounter = 0;
        mv.speedMetersPerSecond = 0.0;
        mv.headingDeg = 0.0;
        mv.pitchDeg = 0.0;
        mv.flamePhase = 0.0;
    }

    rebuildThreatGeometry();
    for (int i = 0; i < static_cast<int>(m_missileVisuals.size()); ++i) {
        rebuildMissileMarker(i);
        rebuildMissileRouteGeometry(i);
        rebuildMissileTrailGeometry(i);
        clearMissileImpact(i);
        clearMissile(i);
    }

}

void OsgEarthWidget::zoomIn() {
    zoomBySteps(2);
}

void OsgEarthWidget::zoomOut() {
    zoomBySteps(-2);
}

void OsgEarthWidget::setMissileCount(int count) {
    ensureSceneCreated();
    if (!m_overlayGroup.valid()) {
        return;
    }

    while (static_cast<int>(m_missileVisuals.size()) < count) {
        int idx = static_cast<int>(m_missileVisuals.size());
        MissileVisual mv;
        mv.color = mission::missileColor(idx);

        mv.group = new osg::Group;
        mv.routeGeode = new osg::Geode;
        mv.trailGeode = new osg::Geode;

        mv.missileNode = new osg::MatrixTransform;
        mv.missileNode->setNodeMask(0u);

        osg::ref_ptr<osg::MatrixTransform> exhaustNode;
        osg::ref_ptr<osg::ShapeDrawable> exhaustDrawable;
        osg::ref_ptr<osg::Group> missileModel = buildMissileModel(mv.color, exhaustNode, exhaustDrawable);
        mv.exhaustNode = exhaustNode;
        mv.exhaustDrawable = exhaustDrawable;
        mv.missileNode->addChild(missileModel.get());

        mv.group->addChild(mv.routeGeode.get());
        mv.group->addChild(mv.trailGeode.get());
        mv.group->addChild(mv.missileNode.get());

        m_overlayGroup->addChild(mv.group.get());
        m_missileVisuals.push_back(std::move(mv));
    }

    while (static_cast<int>(m_missileVisuals.size()) > count) {
        auto& mv = m_missileVisuals.back();
        if (mv.group.valid() && m_overlayGroup.valid()) {
            m_overlayGroup->removeChild(mv.group.get());
        }
        m_missileVisuals.pop_back();
    }

    if (m_activeMissileIndex >= static_cast<int>(m_missileVisuals.size())) {
        m_activeMissileIndex = -1;
    }
}

int OsgEarthWidget::missileCount() const {
    return static_cast<int>(m_missileVisuals.size());
}

void OsgEarthWidget::setMissileNames(const std::vector<std::string>& names) {
    m_missileNames = names;
}

void OsgEarthWidget::setMissileStartPoint(int index, const osgEarth::GeoPoint& point) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.startPoint = point;
    mv.hasStart = true;
    rebuildMissileMarker(index);
}

void OsgEarthWidget::setMissileTargetPoint(int index, const osgEarth::GeoPoint& point) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.targetPoint = point;
    mv.hasTarget = true;
    rebuildMissileMarker(index);
}

void OsgEarthWidget::setMissileRoute(int index, const std::vector<osgEarth::GeoPoint>& route) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.route = route;
    mv.trail.clear();
    rebuildMissileRouteGeometry(index);
    rebuildMissileTrailGeometry(index);
    clearMissileImpact(index);
}

void OsgEarthWidget::setMissilePosition(int index, const osgEarth::GeoPoint& position) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];

    if (!mv.missileNode.valid()) {
        return;
    }

    osg::Vec3d world;
    position.toWorld(world);

    osg::Matrixd pose = osg::Matrix::translate(world);
    if (mv.hasMissile && mv.missilePoint.isValid()) {
        osg::Vec3d prevWorld;
        mv.missilePoint.toWorld(prevWorld);
        osg::Vec3d direction = world - prevWorld;
        if (direction.length2() > 1e-6) {
            direction.normalize();
            osg::Quat rotation;
            rotation.makeRotate(osg::Vec3d(0.0, 0.0, 1.0), direction);
            pose = osg::Matrix::rotate(rotation) * osg::Matrix::translate(world);

            mv.headingDeg = headingDegrees(mv.missilePoint, position);
            mv.pitchDeg = pitchDegrees(mv.missilePoint, position);
        }
    }

    mv.previousPoint = mv.missilePoint;
    mv.hasPreviousPoint = mv.hasMissile;
    mv.missilePoint = position;
    mv.hasMissile = true;
    m_activeMissileIndex = index;

    mv.missileNode->setMatrix(pose);
    mv.missileNode->setNodeMask(~0u);

    if (mv.exhaustNode.valid() && mv.exhaustDrawable.valid()) {
        mv.flamePhase += 0.35;
        const double speedRatio = std::clamp(mv.speedMetersPerSecond / 950.0, 0.0, 1.0);
        const double pulse = 0.72 + 0.28 * std::sin(mv.flamePhase);
        const double scale = 0.55 + speedRatio * 0.95;
        mv.exhaustNode->setMatrix(
            osg::Matrix::scale(1.0, 1.0, scale * pulse) *
            osg::Matrix::translate(0.0, 0.0, -3500.0));
        mv.exhaustDrawable->setColor(osg::Vec4(
            1.0f,
            static_cast<float>(0.50 + 0.35 * (1.0 - speedRatio)),
            static_cast<float>(0.12 + 0.22 * (1.0 - speedRatio)),
            static_cast<float>(0.68 + 0.22 * pulse)));
    }

    if (mv.trail.empty() || approximateDistanceMeters(mv.trail.back(), position) > 600.0) {
        mv.trail.push_back(position);
        rebuildMissileTrailGeometry(index);
    }

    if (mv.followEnabled && mv.hasMissile) {
        ++mv.followTickCounter;
        if (mv.followTickCounter >= 3) {
            mv.followTickCounter = 0;
            focusOnPoint(position, 120000.0);
        }
    }

}

void OsgEarthWidget::setMissileTelemetry(int index, double speedMetersPerSecond, double elapsedSeconds) {
    Q_UNUSED(elapsedSeconds);
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.speedMetersPerSecond = std::max(0.0, speedMetersPerSecond);
    if (mv.hasMissile) {
        m_activeMissileIndex = index;
    }
}

void OsgEarthWidget::setMissileColor(int index, const osg::Vec4& color) {
    ensureMissileVisual(index);
    m_missileVisuals[static_cast<std::size_t>(index)].color = color;
}

void OsgEarthWidget::clearMissile(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.hasMissile = false;
    mv.hasPreviousPoint = false;
    mv.speedMetersPerSecond = 0.0;
    mv.followTickCounter = 0;
    if (mv.missileNode.valid()) {
        mv.missileNode->setNodeMask(0u);
    }

    if (m_activeMissileIndex == index) {
        m_activeMissileIndex = -1;
    }
}

void OsgEarthWidget::showMissileImpact(int index, const osgEarth::GeoPoint& point) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];

    if (!mv.group.valid() || !point.isValid()) {
        return;
    }

    clearMissileImpact(index);

    osg::Vec3d world;
    point.toWorld(world);

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
    xform->setMatrix(osg::Matrix::translate(world));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> innerGlow = new osg::ShapeDrawable(
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 3500.0f));
    innerGlow->setColor(osg::Vec4(mv.color.r(), mv.color.g() * 0.5f, 0.1f, 0.95f));

    osg::ref_ptr<osg::ShapeDrawable> outerGlow = new osg::ShapeDrawable(
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 7000.0f));
    outerGlow->setColor(osg::Vec4(mv.color.r(), mv.color.g() * 0.3f, 0.1f, 0.25f));

    geode->addDrawable(innerGlow.get());
    geode->addDrawable(outerGlow.get());

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    xform->addChild(geode.get());
    mv.impactNode = xform;
    mv.group->addChild(mv.impactNode.get());
}

void OsgEarthWidget::clearMissileImpact(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    if (mv.group.valid() && mv.impactNode.valid()) {
        mv.group->removeChild(mv.impactNode.get());
    }
    mv.impactNode = nullptr;
}

void OsgEarthWidget::setFollowMissile(int index, bool enabled) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    m_missileVisuals[static_cast<std::size_t>(index)].followEnabled = enabled;
    m_missileVisuals[static_cast<std::size_t>(index)].followTickCounter = 0;
    if (enabled) {
        m_activeMissileIndex = index;
    }
}

void OsgEarthWidget::focusOnAllRoutes() {
    std::vector<osgEarth::GeoPoint> allPoints;
    for (const auto& mv : m_missileVisuals) {
        if (!mv.route.empty()) {
            allPoints.insert(allPoints.end(), mv.route.begin(), mv.route.end());
        }
    }

    if (allPoints.empty()) {
        return;
    }

    double minLon = allPoints.front().x();
    double maxLon = allPoints.front().x();
    double minLat = allPoints.front().y();
    double maxLat = allPoints.front().y();
    double minAlt = allPoints.front().z();
    double maxAlt = allPoints.front().z();

    for (const auto& p : allPoints) {
        minLon = std::min(minLon, p.x());
        maxLon = std::max(maxLon, p.x());
        minLat = std::min(minLat, p.y());
        maxLat = std::max(maxLat, p.y());
        minAlt = std::min(minAlt, p.z());
        maxAlt = std::max(maxAlt, p.z());
    }

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) {
        return;
    }

    osgEarth::GeoPoint center(
        wgs84,
        (minLon + maxLon) * 0.5,
        (minLat + maxLat) * 0.5,
        (minAlt + maxAlt) * 0.5,
        osgEarth::ALTMODE_ABSOLUTE);

    const double lonSpan = maxLon - minLon;
    const double latSpan = maxLat - minLat;
    const double meanLatRad = toRadians((minLat + maxLat) * 0.5);
    const double metersPerLonDeg = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double horizontalSpan = std::max(lonSpan * metersPerLonDeg, latSpan * kMetersPerLatDegree);
    const double rangeMeters = std::clamp(horizontalSpan * 1.6 + 60000.0, 90000.0, 2000000.0);
    focusOnPoint(center, rangeMeters);
}

QPaintEngine* OsgEarthWidget::paintEngine() const {
    return nullptr;
}

void OsgEarthWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
}

void OsgEarthWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    syncGraphicsWindowSize();
}

void OsgEarthWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    QTimer::singleShot(0, this, [this]() {
        initializeViewer();
        syncGraphicsWindowSize();
    });
    setFocus(Qt::OtherFocusReason);
}

int qtToOsgButton(Qt::MouseButton btn) {
    switch (btn) {
    case Qt::LeftButton: return 1;
    case Qt::MiddleButton: return 2;
    case Qt::RightButton: return 3;
    default: return 0;
    }
}

int qtToOsgKey(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) return osgGA::GUIEventAdapter::KEY_Escape;
    if (event->key() == Qt::Key_Shift) return osgGA::GUIEventAdapter::KEY_Shift_L;
    if (event->key() == Qt::Key_Control) return osgGA::GUIEventAdapter::KEY_Control_L;
    if (event->key() == Qt::Key_Alt) return osgGA::GUIEventAdapter::KEY_Alt_L;
    if (event->text().length() > 0) return event->text().at(0).unicode();
    return event->key();
}

void OsgEarthWidget::mousePressEvent(QMouseEvent* event) {
    if (m_graphicsWindow.valid()) {
        if (!hasFocus()) {
            setFocus(Qt::MouseFocusReason);
        }
        const double dpr = devicePixelRatioF();
        m_graphicsWindow->getEventQueue()->mouseButtonPress(
            static_cast<float>(event->position().x() * dpr),
            static_cast<float>(event->position().y() * dpr),
            qtToOsgButton(event->button()));
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void OsgEarthWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (m_graphicsWindow.valid()) {
        const double dpr = devicePixelRatioF();
        m_graphicsWindow->getEventQueue()->mouseButtonRelease(
            static_cast<float>(event->position().x() * dpr),
            static_cast<float>(event->position().y() * dpr),
            qtToOsgButton(event->button()));
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void OsgEarthWidget::mouseMoveEvent(QMouseEvent* event) {
    if (m_graphicsWindow.valid()) {
        const double dpr = devicePixelRatioF();
        m_graphicsWindow->getEventQueue()->mouseMotion(
            static_cast<float>(event->position().x() * dpr),
            static_cast<float>(event->position().y() * dpr));
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void OsgEarthWidget::wheelEvent(QWheelEvent* event) {
    if (m_graphicsWindow.valid()) {
        const QPoint angle = event->angleDelta();
        const int delta = angle.y();
        if (delta != 0) {
            const int steps = std::max(1, std::abs(delta) / 120);
            zoomBySteps(delta > 0 ? steps : -steps);
        } else {
            const QPoint pixel = event->pixelDelta();
            if (pixel.y() != 0) {
                zoomBySteps(pixel.y() > 0 ? 1 : -1);
            }
        }
        event->accept();
        return;
    }
    QWidget::wheelEvent(event);
}

void OsgEarthWidget::keyPressEvent(QKeyEvent* event) {
    if (m_graphicsWindow.valid()) {
        m_graphicsWindow->getEventQueue()->keyPress(
            static_cast<osgGA::GUIEventAdapter::KeySymbol>(qtToOsgKey(event)));
    }
    QWidget::keyPressEvent(event);
}

void OsgEarthWidget::keyReleaseEvent(QKeyEvent* event) {
    if (m_graphicsWindow.valid()) {
        m_graphicsWindow->getEventQueue()->keyRelease(
            static_cast<osgGA::GUIEventAdapter::KeySymbol>(qtToOsgKey(event)));
    }
    QWidget::keyReleaseEvent(event);
}

void OsgEarthWidget::syncGraphicsWindowSize() {
    if (!m_graphicsWindow.valid() || !m_viewer.valid()) {
        return;
    }

    const double dpr = devicePixelRatioF();
    const int w = scaledPixels(width(), dpr);
    const int h = scaledPixels(height(), dpr);
    if (w == m_cachedViewportWidth && h == m_cachedViewportHeight) {
        return;
    }

    m_cachedViewportWidth = w;
    m_cachedViewportHeight = h;

    m_graphicsWindow->resized(0, 0, w, h);
    m_graphicsWindow->getEventQueue()->windowResize(0, 0, w, h);
    m_viewer->getCamera()->setViewport(0, 0, w, h);
    m_viewer->getCamera()->setProjectionMatrixAsPerspective(
        35.0,
        static_cast<double>(w) / static_cast<double>(h),
        1.0,
        50000000.0);
}

void OsgEarthWidget::initializeViewer() {
    if (m_initialized || m_initializing) {
        return;
    }

    m_initializing = true;

    m_viewer = new osgViewer::Viewer;
    m_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    m_viewer->setReleaseContextAtEndOfFrameHint(false);

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    const double dpr = devicePixelRatioF();
    traits->width = scaledPixels(width(), dpr);
    traits->height = scaledPixels(height(), dpr);
    traits->windowDecoration = false;
    traits->doubleBuffer = true;
    traits->sharedContext = nullptr;

#ifdef _WIN32
    traits->inheritedWindowData = new osgViewer::GraphicsWindowWin32::WindowData(reinterpret_cast<HWND>(winId()), false);
#endif

    osg::ref_ptr<osg::GraphicsContext> context = osg::GraphicsContext::createGraphicsContext(traits.get());
    m_graphicsWindow = dynamic_cast<osgViewer::GraphicsWindow*>(context.get());
    if (!m_graphicsWindow.valid()) {
        m_viewer = nullptr;
        m_initializing = false;
        return;
    }

    osg::Camera* camera = m_viewer->getCamera();
    camera->setGraphicsContext(m_graphicsWindow.get());
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setProjectionMatrixAsPerspective(
        35.0,
        static_cast<double>(traits->width) / static_cast<double>(traits->height),
        1.0,
        50000000.0);
    camera->setClearColor(osg::Vec4(0.03f, 0.05f, 0.08f, 1.0f));

    m_earthManipulator = new osgEarth::Util::EarthManipulator;
    m_trackballManipulator = new osgGA::TrackballManipulator;

    ensureSceneCreated();
    m_viewer->setSceneData(m_root.get());
    updateCameraManipulator();

    m_cachedViewportWidth = 0;
    m_cachedViewportHeight = 0;
    syncGraphicsWindowSize();

    m_initialized = true;
    m_initializing = false;
}

void OsgEarthWidget::ensureSceneCreated() {
    if (m_root.valid()) {
        return;
    }

    m_sceneCreationAttempted = true;

    m_root = new osg::Group;
    m_realEarthGroup = new osg::Group;
    m_fallbackGroup = new osg::Group;
    m_hasRealEarthDataset = false;
    m_realEarthFromLocalFile = false;
    m_realEarthSourcePath.clear();
    m_mapNode = nullptr;

    std::string loadedEarthPath;
    osg::ref_ptr<osg::Node> earthNode = tryLoadEarthFile(&loadedEarthPath);
    if (earthNode.valid()) {
        m_realEarthGroup->addChild(earthNode.get());
        m_mapNode = osgEarth::MapNode::findMapNode(earthNode.get());
        m_hasRealEarthDataset = m_mapNode.valid();
        if (m_hasRealEarthDataset) {
            m_realEarthFromLocalFile = true;
            m_realEarthSourcePath = QString::fromStdString(loadedEarthPath);
        }
    } else {
        osg::ref_ptr<osgEarth::MapNode> onlineMapNode;
        osg::ref_ptr<osg::Node> onlineEarth = buildOnlineRealEarthNode(onlineMapNode);
        if (onlineEarth.valid()) {
            m_realEarthGroup->addChild(onlineEarth.get());
            m_mapNode = onlineMapNode;
            m_hasRealEarthDataset = true;
            m_realEarthSourcePath = QStringLiteral("online-xyz");
        }
    }

    m_root->addChild(m_realEarthGroup.get());
    m_root->addChild(m_fallbackGroup.get());
    rebuildFallbackGlobe();
    applyGlobeMode();

    m_overlayGroup = new osg::Group;
    m_threatGroup = new osg::Group;
    m_markerGroup = new osg::Group;

    m_overlayGroup->addChild(m_threatGroup.get());

    m_root->addChild(m_overlayGroup.get());

    rebuildThreatGeometry();
}

void OsgEarthWidget::rebuildFallbackGlobe() {
    if (!m_fallbackGroup.valid()) {
        return;
    }

    const bool presentationMode = m_globeMode == GlobeMode::Presentation;

    m_fallbackGroup->removeChildren(0, m_fallbackGroup->getNumChildren());

    m_fallbackGroup->addChild(buildStarFieldGeode().get());

    osg::ref_ptr<osg::Geode> globeGeode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> globe = new osg::ShapeDrawable(
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), kFallbackEarthRadiusMeters));
    globe->setColor(presentationMode
                        ? osg::Vec4(0.07f, 0.24f, 0.47f, 1.0f)
                        : osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    globeGeode->addDrawable(globe.get());

    osg::StateSet* globeState = globeGeode->getOrCreateStateSet();
    globeState->setMode(GL_LIGHTING, presentationMode ? osg::StateAttribute::OFF : osg::StateAttribute::ON);

    if (!presentationMode) {
        osg::ref_ptr<osg::Image> earthTexture = tryLoadEarthTextureImage();
        if (!earthTexture.valid()) {
            earthTexture = buildProceduralEarthTexture();
        }

        if (earthTexture.valid()) {
            osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(earthTexture.get());
            tex->setDataVariance(osg::Object::STATIC);
            tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
            tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
            tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            globeState->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
        }
    }

    osg::ref_ptr<osg::Geode> atmosphereGeode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> atmosphere = new osg::ShapeDrawable(
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), kFallbackEarthRadiusMeters + 80000.0));
    atmosphere->setColor(presentationMode
                             ? osg::Vec4(0.38f, 0.72f, 1.0f, 0.18f)
                             : osg::Vec4(0.38f, 0.62f, 1.0f, 0.13f));
    atmosphereGeode->addDrawable(atmosphere.get());

    osg::StateSet* atmState = atmosphereGeode->getOrCreateStateSet();
    atmState->setMode(GL_BLEND, osg::StateAttribute::ON);
    atmState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    atmState->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    osg::ref_ptr<osg::Geode> graticuleGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> graticule = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;

    constexpr double gridAltitude = 16000.0;
    const double radius = kFallbackEarthRadiusMeters + gridAltitude;

    const int majorStep = presentationMode ? 15 : 30;
    const int lonSubStep = presentationMode ? 3 : 6;
    const int latSubStep = presentationMode ? 2 : 4;

    for (int lat = -75; lat <= 75; lat += majorStep) {
        const int startIndex = static_cast<int>(vertices->size());
        for (int lon = -180; lon <= 180; lon += lonSubStep) {
            vertices->push_back(geodeticToWorld(static_cast<double>(lon), static_cast<double>(lat), radius));
        }
        graticule->addPrimitiveSet(new osg::DrawArrays(
            GL_LINE_STRIP,
            startIndex,
            static_cast<GLsizei>(vertices->size() - startIndex)));
    }

    for (int lon = -180; lon <= 180; lon += majorStep) {
        const int startIndex = static_cast<int>(vertices->size());
        for (int lat = -90; lat <= 90; lat += latSubStep) {
            vertices->push_back(geodeticToWorld(static_cast<double>(lon), static_cast<double>(lat), radius));
        }
        graticule->addPrimitiveSet(new osg::DrawArrays(
            GL_LINE_STRIP,
            startIndex,
            static_cast<GLsizei>(vertices->size() - startIndex)));
    }

    graticule->setVertexArray(vertices.get());
    colors->push_back(presentationMode
                          ? osg::Vec4(0.28f, 0.80f, 1.0f, 0.35f)
                          : osg::Vec4(0.25f, 0.73f, 1.0f, 0.15f));
    graticule->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

    osg::StateSet* gridState = graticule->getOrCreateStateSet();
    gridState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    gridState->setMode(GL_BLEND, osg::StateAttribute::ON);
    gridState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::LineWidth> gridWidth = new osg::LineWidth(presentationMode ? 1.7f : 1.0f);
    gridState->setAttributeAndModes(gridWidth.get(), osg::StateAttribute::ON);

    graticuleGeode->addDrawable(graticule.get());

    m_fallbackGroup->addChild(globeGeode.get());
    m_fallbackGroup->addChild(atmosphereGeode.get());
    m_fallbackGroup->addChild(graticuleGeode.get());
}

void OsgEarthWidget::applyGlobeMode() {
    if (!m_realEarthGroup.valid() || !m_fallbackGroup.valid()) {
        return;
    }

    const bool wantRealistic = m_globeMode == GlobeMode::Realistic;
    const bool showRealDataset = wantRealistic && m_hasRealEarthDataset;

    m_realEarthGroup->setNodeMask(showRealDataset ? ~0u : 0u);
    m_fallbackGroup->setNodeMask(showRealDataset ? 0u : ~0u);
    updateCameraManipulator();
}

void OsgEarthWidget::updateCameraManipulator() {
    if (!m_viewer.valid()) {
        return;
    }

    const bool useEarthManipulator =
        m_globeMode == GlobeMode::Realistic && m_hasRealEarthDataset && m_earthManipulator.valid();

    if (useEarthManipulator) {
        m_viewer->setCameraManipulator(m_earthManipulator.get(), false);
        return;
    }

    if (m_trackballManipulator.valid()) {
        m_viewer->setCameraManipulator(m_trackballManipulator.get(), false);
    }
}

void OsgEarthWidget::rebuildThreatGeometry() {
    ensureSceneCreated();
    if (!m_threatGroup.valid()) {
        return;
    }

    while (m_threatVisualNodes.size() > m_threats.size()) {
        osg::ref_ptr<osg::MatrixTransform> node = m_threatVisualNodes.back();
        if (node.valid()) {
            m_threatGroup->removeChild(node.get());
        }
        m_threatVisualNodes.pop_back();
    }

    while (m_threatVisualNodes.size() < m_threats.size()) {
        m_threatVisualNodes.push_back(nullptr);
    }

    for (std::size_t i = 0; i < m_threats.size(); ++i) {
        osg::ref_ptr<osg::MatrixTransform> newNode = buildThreatVisualNode(m_threats[i]);
        if (m_threatVisualNodes[i].valid()) {
            m_threatGroup->removeChild(m_threatVisualNodes[i].get());
        }
        m_threatVisualNodes[i] = newNode;
        m_threatGroup->addChild(newNode.get());
    }
}

void OsgEarthWidget::rebuildMarkers() {
}

void OsgEarthWidget::rebuildRouteGeometry() {
}

void OsgEarthWidget::rebuildTrailGeometry() {
}

void OsgEarthWidget::ensureMissileVisual(int index) {
    if (index < 0) {
        return;
    }
    if (index >= static_cast<int>(m_missileVisuals.size())) {
        setMissileCount(index + 1);
    }
}

void OsgEarthWidget::rebuildMissileRouteGeometry(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    if (!mv.routeGeode.valid()) {
        return;
    }

    mv.routeGeode->removeDrawables(0, mv.routeGeode->getNumDrawables());

    if (mv.route.size() < 2) {
        return;
    }

    double minAltitude = mv.route.front().z();
    double maxAltitude = mv.route.front().z();
    for (const auto& point : mv.route) {
        minAltitude = std::min(minAltitude, point.z());
        maxAltitude = std::max(maxAltitude, point.z());
    }
    const double altitudeSpan = std::max(1.0, maxAltitude - minAltitude);

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(mv.route.size());

    osg::ref_ptr<osg::Vec4Array> routeColors = new osg::Vec4Array;
    routeColors->reserve(mv.route.size());

    osg::ref_ptr<osg::Vec3Array> shadowVertices = new osg::Vec3Array;
    shadowVertices->reserve(mv.route.size());

    osg::ref_ptr<osg::Vec3Array> altitudeVertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> altitudeColors = new osg::Vec4Array;

    const std::size_t sampleStep = std::max<std::size_t>(1, mv.route.size() / 10);

    const osg::Vec4 baseColor = mv.color;
    const osg::Vec4 brightColor(
        std::min(1.0f, baseColor.r() + 0.3f),
        std::min(1.0f, baseColor.g() + 0.3f),
        std::min(1.0f, baseColor.b() + 0.3f),
        1.0f);

    for (std::size_t i = 0; i < mv.route.size(); ++i) {
        const auto& point = mv.route[i];
        osg::Vec3d world;
        point.toWorld(world);
        vertices->push_back(world);

        const double altitudeT = clamp01((point.z() - minAltitude) / altitudeSpan);
        routeColors->push_back(mixColor(baseColor, brightColor, altitudeT));

        const auto* wgs84 = point.getSRS() != nullptr ? point.getSRS() : osgEarth::SpatialReference::get("wgs84");
        if (wgs84 == nullptr) {
            continue;
        }

        osgEarth::GeoPoint shadowPoint(wgs84, point.x(), point.y(), 0.0, osgEarth::ALTMODE_ABSOLUTE);
        osg::Vec3d shadowWorld;
        shadowPoint.toWorld(shadowWorld);
        shadowVertices->push_back(shadowWorld);

        if (i == 0 || i + 1 == mv.route.size() || (i % sampleStep) == 0) {
            altitudeVertices->push_back(shadowWorld);
            altitudeVertices->push_back(world);
            altitudeColors->push_back(osg::Vec4(baseColor.r() * 0.5f, baseColor.g() * 0.5f, baseColor.b() * 0.5f, 0.20f));
            altitudeColors->push_back(osg::Vec4(baseColor.r(), baseColor.g(), baseColor.b(), 0.85f));
        }
    }

    osg::ref_ptr<osg::Geometry> shadowLine = new osg::Geometry;
    shadowLine->setVertexArray(shadowVertices.get());
    shadowLine->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(shadowVertices->size())));
    osg::ref_ptr<osg::Vec4Array> shadowColors = new osg::Vec4Array;
    shadowColors->push_back(osg::Vec4(baseColor.r() * 0.3f, baseColor.g() * 0.3f, baseColor.b() * 0.3f, 0.35f));
    shadowLine->setColorArray(shadowColors.get(), osg::Array::BIND_OVERALL);
    osg::StateSet* shadowState = shadowLine->getOrCreateStateSet();
    shadowState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    shadowState->setMode(GL_BLEND, osg::StateAttribute::ON);
    shadowState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> shadowWidth = new osg::LineWidth(3.0f);
    shadowState->setAttributeAndModes(shadowWidth.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geometry> routeLine = new osg::Geometry;
    routeLine->setVertexArray(vertices.get());
    routeLine->setColorArray(routeColors.get(), osg::Array::BIND_PER_VERTEX);
    routeLine->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(vertices->size())));
    osg::StateSet* routeState = routeLine->getOrCreateStateSet();
    routeState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    routeState->setMode(GL_BLEND, osg::StateAttribute::ON);
    routeState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> routeWidth = new osg::LineWidth(6.0f);
    routeState->setAttributeAndModes(routeWidth.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geometry> altitudePosts = new osg::Geometry;
    altitudePosts->setVertexArray(altitudeVertices.get());
    altitudePosts->setColorArray(altitudeColors.get(), osg::Array::BIND_PER_VERTEX);
    altitudePosts->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(altitudeVertices->size())));
    osg::StateSet* postsState = altitudePosts->getOrCreateStateSet();
    postsState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    postsState->setMode(GL_BLEND, osg::StateAttribute::ON);
    postsState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> postsWidth = new osg::LineWidth(2.0f);
    postsState->setAttributeAndModes(postsWidth.get(), osg::StateAttribute::ON);

    mv.routeGeode->addDrawable(shadowLine.get());
    mv.routeGeode->addDrawable(routeLine.get());
    if (!altitudeVertices->empty()) {
        mv.routeGeode->addDrawable(altitudePosts.get());
    }
}

void OsgEarthWidget::rebuildMissileTrailGeometry(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    if (!mv.trailGeode.valid()) {
        return;
    }

    mv.trailGeode->removeDrawables(0, mv.trailGeode->getNumDrawables());
    if (mv.trail.size() < 2) {
        return;
    }

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(mv.trail.size());

    for (const auto& point : mv.trail) {
        osg::Vec3d world;
        point.toWorld(world);
        vertices->push_back(world);
    }

    osg::ref_ptr<osg::Geometry> line = new osg::Geometry;
    line->setVertexArray(vertices.get());
    line->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(vertices->size())));

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    const osg::Vec4 trailColor(mv.color.r(), mv.color.g(), mv.color.b(), 0.85f);
    colors->push_back(trailColor);
    line->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

    osg::StateSet* stateSet = line->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth(5.0f);
    stateSet->setAttributeAndModes(width.get(), osg::StateAttribute::ON);

    mv.trailGeode->addDrawable(line.get());
}

void OsgEarthWidget::rebuildMissileMarker(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    if (!mv.group.valid()) {
        return;
    }

    if (mv.startMarker.valid()) {
        mv.group->removeChild(mv.startMarker.get());
        mv.startMarker = nullptr;
    }
    if (mv.targetMarker.valid()) {
        mv.group->removeChild(mv.targetMarker.get());
        mv.targetMarker = nullptr;
    }

    const osg::Vec4 startColor(
        std::min(1.0f, mv.color.r() * 0.6f + 0.4f),
        std::min(1.0f, mv.color.g() * 0.6f + 0.4f),
        std::min(1.0f, mv.color.b() * 0.6f + 0.4f),
        0.92f);

    const osg::Vec4 targetColor(
        std::min(1.0f, mv.color.r() + 0.2f),
        std::min(1.0f, mv.color.g() + 0.2f),
        std::min(1.0f, mv.color.b() + 0.2f),
        0.92f);

    if (mv.hasStart) {
        mv.startMarker = buildVerticalMarkerNode(mv.startPoint, startColor, 2400.0);
        mv.group->addChild(mv.startMarker.get());
    }

    if (mv.hasTarget) {
        mv.targetMarker = buildVerticalMarkerNode(mv.targetPoint, targetColor, 2400.0);
        mv.group->addChild(mv.targetMarker.get());
    }
}
