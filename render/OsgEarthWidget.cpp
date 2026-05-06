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
#include <QDebug>
#include <QDateTime>
#include <QToolTip>

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
#include <osg/Vec4d>
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
#include <osgEarth/Registry>
#include <osgEarth/SpatialReference>
#include <osgEarth/TileKey>
#include <osgEarth/TileLayer>
#include <osgEarth/ElevationLayer>
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
osgEarth::GeoPoint lerpGeoPoint(const osgEarth::GeoPoint& a, const osgEarth::GeoPoint& b, double t) {
    const auto* srs = a.getSRS() != nullptr ? a.getSRS() : osgEarth::SpatialReference::get("wgs84");
    return osgEarth::GeoPoint(
        srs,
        a.x() + (b.x() - a.x()) * t,
        a.y() + (b.y() - a.y()) * t,
        a.z() + (b.z() - a.z()) * t,
        osgEarth::ALTMODE_ABSOLUTE);
}

std::vector<osgEarth::GeoPoint> smoothPolylineForRender(
    const std::vector<osgEarth::GeoPoint>& points,
    double cornerRatio,
    int cornerSamples) {
    if (points.size() < 3) {
        return points;
    }

    const double r = std::clamp(cornerRatio, 0.08, 0.32);
    const int samples = std::clamp(cornerSamples, 3, 12);

    std::vector<osgEarth::GeoPoint> out;
    out.reserve(points.size() * static_cast<std::size_t>(samples));
    out.push_back(points.front());

    osgEarth::GeoPoint prevExit = points.front();
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const auto& prev = points[i - 1];
        const auto& curr = points[i];
        const auto& next = points[i + 1];

        const osgEarth::GeoPoint entry = lerpGeoPoint(curr, prev, r);
        const osgEarth::GeoPoint exit = lerpGeoPoint(curr, next, r);

        if (approximateDistanceMeters(prevExit, entry) > 1.0) {
            out.push_back(entry);
        }

        for (int k = 1; k < samples; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(samples);
            const osgEarth::GeoPoint a = lerpGeoPoint(entry, curr, t);
            const osgEarth::GeoPoint b = lerpGeoPoint(curr, exit, t);
            out.push_back(lerpGeoPoint(a, b, t));
        }

        out.push_back(exit);
        prevExit = exit;
    }

    out.push_back(points.back());
    return out;
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
    const osg::Vec4 hotCore = mixColor(osg::Vec4(1.0f, 0.88f, 0.18f, 0.82f), osg::Vec4(1.0f, 0.14f, 0.06f, 0.96f), threatScale);
    const osg::Vec4 coreColor(hotCore.r(), hotCore.g(), hotCore.b(), 0.56f);
    const osg::Vec4 shellColor(1.0f, 0.35f + static_cast<float>(0.20 * threatScale), 0.06f, 0.30f);
    const osg::Vec4 hazeColor(1.0f, 0.20f, 0.08f, 0.14f);
    const osg::Vec4 ringColor(1.0f, 0.95f, 0.30f, 0.58f);
    const osg::Vec4 pillarColor(1.0f, 0.97f, 0.40f, 0.86f);

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

    addCylinder(threat.radiusMeters * 1.28, height * 0.96, hazeColor, 0.0);
    addCylinder(threat.radiusMeters * 0.96, height * 1.02, shellColor, 0.0);
    addCylinder(threat.radiusMeters * 0.58, height * 1.08, coreColor, 0.0);
    addCylinder(std::max(180.0, threat.radiusMeters * 0.13), height * 1.22, pillarColor, 0.0);

    osg::ref_ptr<osg::Geode> topCapGeode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> topCap = new osg::ShapeDrawable(
        new osg::Cone(
            osg::Vec3(0.0f, 0.0f, static_cast<float>(height * 0.66)),
            static_cast<float>(std::max(2200.0, threat.radiusMeters * 0.22)),
            static_cast<float>(std::max(900.0, height * 0.24))));
    topCap->setColor(osg::Vec4(1.0f, 0.97f, 0.48f, 0.80f));
    topCapGeode->addDrawable(topCap.get());
    osg::StateSet* topCapState = topCapGeode->getOrCreateStateSet();
    topCapState->setMode(GL_BLEND, osg::StateAttribute::ON);
    topCapState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    topCapState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    topCapState->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    xform->addChild(topCapGeode.get());

    xform->addChild(buildRingGeode(threat.radiusMeters * 1.28, static_cast<double>(height) * 0.52, ringColor).get());
    xform->addChild(buildRingGeode(threat.radiusMeters * 1.28, -static_cast<double>(height) * 0.52, ringColor).get());
    xform->addChild(buildRingGeode(threat.radiusMeters * 0.78, static_cast<double>(height) * 0.30, osg::Vec4(1.0f, 0.86f, 0.20f, 0.45f)).get());

    return xform;
}

osg::ref_ptr<osg::Node> buildVerticalMarkerNode(const osgEarth::GeoPoint& point, const osg::Vec4& color, double radiusMeters) {
    osg::Vec3d world;
    point.toWorld(world);

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
    xform->setMatrix(makeUpAlignedMatrix(world));

    const double mastHeight = std::max(4500.0, point.z() * 0.95 + 2200.0);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> baseRing = new osg::ShapeDrawable(
        new osg::Cylinder(
            osg::Vec3(0.0f, 0.0f, 0.0f),
            static_cast<float>(radiusMeters * 1.62),
            static_cast<float>(std::max(700.0, radiusMeters * 0.34))));
    baseRing->setColor(osg::Vec4(color.r() * 0.72f, color.g() * 0.72f, color.b() * 0.72f, 0.58f));
    geode->addDrawable(baseRing.get());

    osg::ref_ptr<osg::ShapeDrawable> mast = new osg::ShapeDrawable(
        new osg::Cylinder(
            osg::Vec3(0.0f, 0.0f, static_cast<float>(mastHeight * 0.5)),
            static_cast<float>(radiusMeters * 0.62),
            static_cast<float>(mastHeight)));
    mast->setColor(osg::Vec4(color.r(), color.g(), color.b(), std::min(0.98f, color.a() + 0.05f)));
    geode->addDrawable(mast.get());

    osg::ref_ptr<osg::ShapeDrawable> cap = new osg::ShapeDrawable(
        new osg::Sphere(
            osg::Vec3(0.0f, 0.0f, static_cast<float>(mastHeight + radiusMeters * 0.62)),
            static_cast<float>(radiusMeters * 1.18)));
    cap->setColor(osg::Vec4(
        std::min(1.0f, color.r() + 0.14f),
        std::min(1.0f, color.g() + 0.14f),
        std::min(1.0f, color.b() + 0.14f),
        0.96f));
    geode->addDrawable(cap.get());

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    xform->addChild(geode.get());
    xform->addChild(buildRingGeode(radiusMeters * 2.15, 0.0, osg::Vec4(color.r(), color.g(), color.b(), 0.62f)).get());
    xform->addChild(buildRingGeode(radiusMeters * 1.35, mastHeight * 0.72, osg::Vec4(color.r(), color.g(), color.b(), 0.55f)).get());
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
    constexpr int width = 4096;
    constexpr int height = 2048;

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
    constexpr float kScale = 1.35f;

    osg::ref_ptr<osg::Geode> bodyGeode = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> fuselage = new osg::ShapeDrawable(
        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, -600.0f * kScale), 620.0f * kScale, 5600.0f * kScale));
    fuselage->setColor(osg::Vec4(
        0.87f,
        0.93f,
        0.98f,
        0.98f));
    bodyGeode->addDrawable(fuselage.get());

    osg::ref_ptr<osg::ShapeDrawable> nose = new osg::ShapeDrawable(
        new osg::Cone(osg::Vec3(0.0f, 0.0f, 2600.0f * kScale), 620.0f * kScale, 1900.0f * kScale));
    nose->setColor(osg::Vec4(0.96f, 0.98f, 1.0f, 1.0f));
    bodyGeode->addDrawable(nose.get());

    osg::ref_ptr<osg::ShapeDrawable> ring = new osg::ShapeDrawable(
        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 1200.0f * kScale), 680.0f * kScale, 260.0f * kScale));
    ring->setColor(osg::Vec4(0.26f, 0.42f, 0.62f, 1.0f));
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
        finVertices->push_back(osg::Vec3(720.0f * kScale * c, 720.0f * kScale * s, -1500.0f * kScale));
        finVertices->push_back(osg::Vec3(2050.0f * kScale * c, 2050.0f * kScale * s, -1600.0f * kScale));
        finVertices->push_back(osg::Vec3(1250.0f * kScale * c, 1250.0f * kScale * s, -2600.0f * kScale));

        osg::ref_ptr<osg::Geometry> fin = new osg::Geometry;
        fin->setVertexArray(finVertices.get());
        fin->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, 3));

        osg::ref_ptr<osg::Vec4Array> finColors = new osg::Vec4Array;
        finColors->push_back(osg::Vec4(0.24f, 0.38f, 0.56f, 0.95f));
        fin->setColorArray(finColors.get(), osg::Array::BIND_OVERALL);
        finGeode->addDrawable(fin.get());
    };

    addFin(0.0);
    addFin(3.14159265358979323846 * 0.5);
    addFin(3.14159265358979323846);
    addFin(3.14159265358979323846 * 1.5);
    root->addChild(finGeode.get());

    outExhaustNode = new osg::MatrixTransform;
    outExhaustNode->setMatrix(osg::Matrix::translate(0.0, 0.0, -3500.0 * kScale));

    osg::ref_ptr<osg::Geode> exhaustGeode = new osg::Geode;
    outExhaustDrawable = new osg::ShapeDrawable(
        new osg::Cone(osg::Vec3(0.0f, 0.0f, -450.0f * kScale), 460.0f * kScale, 2200.0f * kScale));
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
    setToolTip(QString());

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

void OsgEarthWidget::updateFollowView(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileVisuals.size())) {
        return;
    }

    const auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    if (!mv.hasMissile || !mv.missilePoint.isValid()) {
        return;
    }

    double terrainMeters = 0.0;
    const bool hasTerrain = sampleTerrainElevationMeters(mv.missilePoint.x(), mv.missilePoint.y(), terrainMeters);
    const double aglMeters = hasTerrain ? std::max(0.0, mv.missilePoint.z() - terrainMeters) : 1200.0;

    const double followRange = std::clamp(36000.0 + aglMeters * 11.0, 28000.0, 320000.0);
    const double followPitch = std::clamp(-20.0 - aglMeters / 5500.0, -42.0, -20.0);

    const double centerAlt = hasTerrain
        ? std::max(terrainMeters + 25.0, mv.missilePoint.z() - std::max(220.0, aglMeters * 0.72))
        : mv.missilePoint.z();
    const double heading = mv.hasPreviousPoint ? mv.headingDeg : 20.0;

    osgEarth::GeoPoint followCenter = mv.missilePoint;
    followCenter.z() = centerAlt;

    if (m_globeMode == GlobeMode::Realistic && m_hasRealEarthDataset && m_earthManipulator.valid()) {
        osgEarth::Viewpoint vp(
            "missile-follow",
            followCenter.x(), followCenter.y(), followCenter.z(),
            heading, followPitch, followRange);
        m_earthManipulator->setViewpoint(vp, 0.12);
        return;
    }

    focusOnPoint(followCenter, followRange);
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

    osgEarth::Registry::shaderGenerator().run(m_overlayGroup.get(), "mission-overlay");

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
    mv.startPoint.z() = clampAltitudeAboveTerrain(point.x(), point.y(), point.z(), 80.0);
    mv.hasStart = true;
    rebuildMissileMarker(index);
}

void OsgEarthWidget::setMissileTargetPoint(int index, const osgEarth::GeoPoint& point) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.targetPoint = point;
    mv.targetPoint.z() = clampAltitudeAboveTerrain(point.x(), point.y(), point.z(), 80.0);
    mv.hasTarget = true;
    rebuildMissileMarker(index);
}

void OsgEarthWidget::setMissileRoute(int index, const std::vector<osgEarth::GeoPoint>& route) {
    ensureMissileVisual(index);
    auto& mv = m_missileVisuals[static_cast<std::size_t>(index)];
    mv.route.clear();
    mv.route.reserve(route.size());
    for (const auto& point : route) {
        mv.route.push_back(point);
    }
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

    osgEarth::GeoPoint adjustedPosition = position;

    osg::Vec3d world;
    adjustedPosition.toWorld(world);

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

            mv.headingDeg = headingDegrees(mv.missilePoint, adjustedPosition);
            mv.pitchDeg = pitchDegrees(mv.missilePoint, adjustedPosition);
        }
    }

    mv.previousPoint = mv.missilePoint;
    mv.hasPreviousPoint = mv.hasMissile;
    mv.missilePoint = adjustedPosition;
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

    if (mv.trail.empty() || approximateDistanceMeters(mv.trail.back(), adjustedPosition) > 600.0) {
        mv.trail.push_back(adjustedPosition);
        rebuildMissileTrailGeometry(index);
    }

    if (mv.followEnabled && mv.hasMissile) {
        ++mv.followTickCounter;
        if (mv.followTickCounter >= 2) {
            mv.followTickCounter = 0;
            updateFollowView(index);
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
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 5600.0f));
    innerGlow->setColor(osg::Vec4(mv.color.r(), mv.color.g() * 0.5f, 0.1f, 0.95f));

    osg::ref_ptr<osg::ShapeDrawable> outerGlow = new osg::ShapeDrawable(
        new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 12500.0f));
    outerGlow->setColor(osg::Vec4(mv.color.r(), mv.color.g() * 0.3f, 0.1f, 0.25f));

    osg::ref_ptr<osg::ShapeDrawable> blastRing = new osg::ShapeDrawable(
        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.0f), 18000.0f, 1200.0f));
    blastRing->setColor(osg::Vec4(1.0f, 0.72f, 0.26f, 0.18f));

    geode->addDrawable(innerGlow.get());
    geode->addDrawable(outerGlow.get());
    geode->addDrawable(blastRing.get());

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
        updateFollowView(index);
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

void OsgEarthWidget::focusOnMissionArea() {
    if (!m_initialized || m_initializing || !m_viewer.valid()) {
        m_pendingMissionFocus = true;
        return;
    }

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) {
        return;
    }

    bool hasAny = false;
    double minLon = 0.0;
    double maxLon = 0.0;
    double minLat = 0.0;
    double maxLat = 0.0;
    double minAlt = 0.0;
    double maxAlt = 0.0;
    double threatRadiusMax = 0.0;

    auto includePoint = [&](double lon, double lat, double alt) {
        if (!hasAny) {
            minLon = maxLon = lon;
            minLat = maxLat = lat;
            minAlt = maxAlt = alt;
            hasAny = true;
            return;
        }
        minLon = std::min(minLon, lon);
        maxLon = std::max(maxLon, lon);
        minLat = std::min(minLat, lat);
        maxLat = std::max(maxLat, lat);
        minAlt = std::min(minAlt, alt);
        maxAlt = std::max(maxAlt, alt);
    };

    for (const auto& mv : m_missileVisuals) {
        if (mv.hasStart && mv.startPoint.isValid()) {
            includePoint(mv.startPoint.x(), mv.startPoint.y(), mv.startPoint.z());
        }
        if (mv.hasTarget && mv.targetPoint.isValid()) {
            includePoint(mv.targetPoint.x(), mv.targetPoint.y(), mv.targetPoint.z());
        }
        for (const auto& p : mv.route) {
            includePoint(p.x(), p.y(), p.z());
        }
    }

    for (const auto& threat : m_threats) {
        includePoint(threat.longitudeDeg, threat.latitudeDeg, (threat.minAltitudeMeters + threat.maxAltitudeMeters) * 0.5);
        threatRadiusMax = std::max(threatRadiusMax, threat.radiusMeters);
        const double lonRadiusDeg = threat.radiusMeters /
            (kMetersPerLatDegree * std::max(0.1, std::cos(toRadians(threat.latitudeDeg))));
        const double latRadiusDeg = threat.radiusMeters / kMetersPerLatDegree;
        includePoint(threat.longitudeDeg - lonRadiusDeg, threat.latitudeDeg - latRadiusDeg, threat.minAltitudeMeters);
        includePoint(threat.longitudeDeg + lonRadiusDeg, threat.latitudeDeg + latRadiusDeg, threat.maxAltitudeMeters);
    }

    if (!hasAny) {
        return;
    }

    osgEarth::GeoPoint center(
        wgs84,
        (minLon + maxLon) * 0.5,
        (minLat + maxLat) * 0.5,
        (minAlt + maxAlt) * 0.5,
        osgEarth::ALTMODE_ABSOLUTE);

    const double meanLatRad = toRadians((minLat + maxLat) * 0.5);
    const double metersPerLonDeg = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double lonSpanMeters = std::abs(maxLon - minLon) * metersPerLonDeg;
    const double latSpanMeters = std::abs(maxLat - minLat) * kMetersPerLatDegree;
    const double horizontalSpan = std::max(lonSpanMeters, latSpanMeters) + threatRadiusMax * 1.4;
    const double altitudeSpan = std::abs(maxAlt - minAlt);
    const double rangeMeters = std::clamp(horizontalSpan * 1.5 + altitudeSpan * 4.0 + 70000.0, 120000.0, 2200000.0);
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
        m_mouseDragging = true;
        if (!hasFocus()) {
            setFocus(Qt::MouseFocusReason);
        }
        float x = 0.0f;
        float y = 0.0f;
        if (mapWidgetToOsgEventCoords(event->position(), x, y)) {
            m_graphicsWindow->getEventQueue()->mouseButtonPress(
                x,
                y,
                qtToOsgButton(event->button()));
        }
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void OsgEarthWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (m_graphicsWindow.valid()) {
        m_mouseDragging = false;
        float x = 0.0f;
        float y = 0.0f;
        if (mapWidgetToOsgEventCoords(event->position(), x, y)) {
            m_graphicsWindow->getEventQueue()->mouseButtonRelease(
                x,
                y,
                qtToOsgButton(event->button()));
        }
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void OsgEarthWidget::mouseMoveEvent(QMouseEvent* event) {
    if (m_graphicsWindow.valid()) {
        float x = 0.0f;
        float y = 0.0f;
        if (mapWidgetToOsgEventCoords(event->position(), x, y)) {
            m_graphicsWindow->getEventQueue()->mouseMotion(x, y);
        }

        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        if (!m_mouseDragging && nowMs - m_lastHoverUpdateMs >= 70) {
            m_lastHoverUpdateMs = nowMs;
            updateHoverTooltip(event->position().toPoint());
        }

        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void OsgEarthWidget::leaveEvent(QEvent* event) {
    m_mouseDragging = false;
    if (!m_lastHoverTooltip.isEmpty()) {
        QToolTip::hideText();
        m_lastHoverTooltip.clear();
    }
    QWidget::leaveEvent(event);
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

bool OsgEarthWidget::mapWidgetToOsgEventCoords(const QPointF& widgetPos, float& outX, float& outY) const {
    const int widgetW = std::max(1, width());
    const int widgetH = std::max(1, height());
    const int viewportW = std::max(1, m_cachedViewportWidth);
    const int viewportH = std::max(1, m_cachedViewportHeight);

    outX = static_cast<float>(widgetPos.x() * static_cast<double>(viewportW) / static_cast<double>(widgetW));
    outY = static_cast<float>(widgetPos.y() * static_cast<double>(viewportH) / static_cast<double>(widgetH));
    return true;
}

bool OsgEarthWidget::projectGeoToScreen(const osgEarth::GeoPoint& point, osg::Vec2d& outScreenPx) const {
    if (!m_viewer.valid() || !point.isValid()) {
        return false;
    }

    osg::Camera* camera = m_viewer->getCamera();
    if (camera == nullptr || camera->getViewport() == nullptr) {
        return false;
    }

    osg::Vec3d world;
    point.toWorld(world);

    const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
    const osg::Vec4d clip = osg::Vec4d(world.x(), world.y(), world.z(), 1.0) * mvp;
    if (std::abs(clip.w()) < 1e-8) {
        return false;
    }

    const double ndcX = clip.x() / clip.w();
    const double ndcY = clip.y() / clip.w();
    const osg::Viewport* vp = camera->getViewport();
    const double xPx = vp->x() + (ndcX + 1.0) * 0.5 * vp->width();
    const double yBottom = vp->y() + (ndcY + 1.0) * 0.5 * vp->height();
    const double yTop = (vp->y() + vp->height()) - yBottom;

    outScreenPx.set(xPx, yTop);
    return std::isfinite(xPx) && std::isfinite(yTop);
}

void OsgEarthWidget::updateHoverTooltip(const QPoint& logicalPos) {
    if (!m_viewer.valid()) {
        return;
    }

    float mouseX = 0.0f;
    float mouseY = 0.0f;
    mapWidgetToOsgEventCoords(QPointF(logicalPos), mouseX, mouseY);

    const osg::Vec2d mouse(static_cast<double>(mouseX), static_cast<double>(mouseY));
    double bestDist2 = 28.0 * 28.0;
    QString bestText;

    auto considerPoint = [&](const osgEarth::GeoPoint& point, const QString& tip) {
        osg::Vec2d screen;
        if (!projectGeoToScreen(point, screen)) {
            return;
        }
        const double dx = screen.x() - mouse.x();
        const double dy = screen.y() - mouse.y();
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < bestDist2) {
            bestDist2 = dist2;
            bestText = tip;
        }
    };

    for (std::size_t i = 0; i < m_missileVisuals.size(); ++i) {
        const auto& mv = m_missileVisuals[i];
        const QString missileName = i < m_missileNames.size()
            ? QString::fromStdString(m_missileNames[i])
            : QStringLiteral("导弹-%1").arg(static_cast<int>(i + 1));

        if (mv.hasStart && mv.startPoint.isValid()) {
            considerPoint(
                mv.startPoint,
                QStringLiteral("%1\n类型: 发射点\n经纬: %2, %3\n高度: %4 m")
                    .arg(missileName)
                    .arg(mv.startPoint.x(), 0, 'f', 4)
                    .arg(mv.startPoint.y(), 0, 'f', 4)
                    .arg(mv.startPoint.z(), 0, 'f', 0));
        }

        if (mv.hasTarget && mv.targetPoint.isValid()) {
            considerPoint(
                mv.targetPoint,
                QStringLiteral("%1\n类型: 打击目标\n经纬: %2, %3\n高度: %4 m")
                    .arg(missileName)
                    .arg(mv.targetPoint.x(), 0, 'f', 4)
                    .arg(mv.targetPoint.y(), 0, 'f', 4)
                    .arg(mv.targetPoint.z(), 0, 'f', 0));
        }

        if (mv.hasMissile && mv.missilePoint.isValid()) {
            considerPoint(
                mv.missilePoint,
                QStringLiteral("%1\n类型: 在途导弹\n经纬: %2, %3\n高度: %4 m\n速度: %5 m/s")
                    .arg(missileName)
                    .arg(mv.missilePoint.x(), 0, 'f', 4)
                    .arg(mv.missilePoint.y(), 0, 'f', 4)
                    .arg(mv.missilePoint.z(), 0, 'f', 0)
                    .arg(mv.speedMetersPerSecond, 0, 'f', 1));
        }
    }

    for (std::size_t i = 0; i < m_threats.size(); ++i) {
        const auto& threat = m_threats[i];
        double groundMeters = 0.0;
        const bool hasGround = sampleTerrainElevationMeters(threat.longitudeDeg, threat.latitudeDeg, groundMeters);
        const double centerAlt = (threat.minAltitudeMeters + threat.maxAltitudeMeters) * 0.5 + (hasGround ? groundMeters : 0.0);
        const osgEarth::GeoPoint center(
            osgEarth::SpatialReference::get("wgs84"),
            threat.longitudeDeg,
            threat.latitudeDeg,
            centerAlt,
            osgEarth::ALTMODE_ABSOLUTE);

        osg::Vec2d centerScreen;
        if (!projectGeoToScreen(center, centerScreen)) {
            continue;
        }

        const double metersPerLon = kMetersPerLatDegree * std::max(0.1, std::cos(toRadians(threat.latitudeDeg)));
        const double deltaLon = threat.radiusMeters / std::max(1.0, metersPerLon);
        osgEarth::GeoPoint edge(
            osgEarth::SpatialReference::get("wgs84"),
            threat.longitudeDeg + deltaLon,
            threat.latitudeDeg,
            centerAlt,
            osgEarth::ALTMODE_ABSOLUTE);
        osg::Vec2d edgeScreen;
        if (!projectGeoToScreen(edge, edgeScreen)) {
            continue;
        }

        const double radiusPx = std::max(14.0, std::sqrt((edgeScreen.x() - centerScreen.x()) * (edgeScreen.x() - centerScreen.x()) +
                                                         (edgeScreen.y() - centerScreen.y()) * (edgeScreen.y() - centerScreen.y())));
        const double dx = centerScreen.x() - mouse.x();
        const double dy = centerScreen.y() - mouse.y();
        const double dist2 = dx * dx + dy * dy;
        if (dist2 > radiusPx * radiusPx) {
            continue;
        }

        const double score = dist2 / std::max(1.0, radiusPx * radiusPx);
        if (score > bestDist2 / (28.0 * 28.0)) {
            continue;
        }

        const QString threatName = threat.name.empty()
            ? QStringLiteral("威胁区-%1").arg(static_cast<int>(i + 1))
            : QString::fromStdString(threat.name);
        bestDist2 = dist2;
        bestText = QStringLiteral("%1\n类型: 威胁空域\n中心: %2, %3\n半径: %4 m\n高度: %5 - %6 m")
            .arg(threatName)
            .arg(threat.longitudeDeg, 0, 'f', 4)
            .arg(threat.latitudeDeg, 0, 'f', 4)
            .arg(threat.radiusMeters, 0, 'f', 0)
            .arg(threat.minAltitudeMeters, 0, 'f', 0)
            .arg(threat.maxAltitudeMeters, 0, 'f', 0);
    }

    if (bestText.isEmpty()) {
        if (!m_lastHoverTooltip.isEmpty()) {
            QToolTip::hideText();
            m_lastHoverTooltip.clear();
        }
        return;
    }

    if (bestText == m_lastHoverTooltip) {
        return;
    }

    m_lastHoverTooltip = bestText;
    QToolTip::showText(mapToGlobal(logicalPos + QPoint(16, 16)), bestText, this);
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

    // osgEarth terrain shaders require OSG compatibility aliases/uniforms.
    if (osg::State* glState = m_graphicsWindow->getState(); glState != nullptr) {
        glState->setUseVertexAttributeAliasing(true);
        glState->resetVertexAttributeAlias(true);
        glState->setUseModelViewAndProjectionUniforms(true);
    }

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

    if (m_pendingMissionFocus) {
        m_pendingMissionFocus = false;
        QTimer::singleShot(0, this, [this]() {
            focusOnMissionArea();
        });
    }

    m_cachedViewportWidth = 0;
    m_cachedViewportHeight = 0;
    syncGraphicsWindowSize();

    QTimer::singleShot(1500, this, [this]() {
        m_loggedTerrainDiagnostics = false;
        logTerrainDiagnostics();
    });

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
    m_loggedTerrainDiagnostics = false;
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

    osg::StateSet* overlayState = m_overlayGroup->getOrCreateStateSet();
    overlayState->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    overlayState->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    overlayState->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    overlayState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    overlayState->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    m_overlayGroup->addChild(m_threatGroup.get());

    m_root->addChild(m_overlayGroup.get());

    rebuildThreatGeometry();
    logTerrainDiagnostics();
}

void OsgEarthWidget::logTerrainDiagnostics() {
    if (m_loggedTerrainDiagnostics) {
        return;
    }
    m_loggedTerrainDiagnostics = true;

    qInfo().noquote() << QStringLiteral("[TerrainDiag] source=%1").arg(m_realEarthSourcePath);

    const std::array<QString, 4> demCandidates = {
        QStringLiteral("dem/15-K.tif"),
        QStringLiteral("data/earth/dem/15-K.tif"),
        QStringLiteral("./dem/15-K.tif"),
        QStringLiteral("./data/earth/dem/15-K.tif")};

    for (const auto& path : demCandidates) {
        const QFileInfo info(path);
        qInfo().noquote() << QStringLiteral("[TerrainDiag] demCandidate path=%1 exists=%2 size=%3")
                                 .arg(path)
                                 .arg(info.exists() ? QStringLiteral("true") : QStringLiteral("false"))
                                 .arg(info.exists() ? QString::number(info.size()) : QStringLiteral("-1"));
    }

    if (!m_mapNode.valid()) {
        qWarning() << "[TerrainDiag] MapNode is null, no terrain diagnostics available.";
        return;
    }

    osgEarth::Map* map = m_mapNode->getMap();
    if (map == nullptr) {
        qWarning() << "[TerrainDiag] Map is null on MapNode.";
        return;
    }

    osgEarth::LayerVector layers;
    map->getLayers(layers);

    int elevationLayerCount = 0;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        osgEarth::Layer* layer = layers[i].get();
        if (layer == nullptr) {
            continue;
        }

        const auto& status = layer->getStatus();
        const osgEarth::GeoExtent& extent = layer->getExtent();
        QString extentText = QStringLiteral("extent=INVALID");
        if (extent.isValid()) {
            extentText = QStringLiteral("extent=[%1,%2]-[%3,%4]")
                             .arg(extent.xMin(), 0, 'f', 4)
                             .arg(extent.yMin(), 0, 'f', 4)
                             .arg(extent.xMax(), 0, 'f', 4)
                             .arg(extent.yMax(), 0, 'f', 4);
        }

        qInfo().noquote() << QStringLiteral("[TerrainDiag] layer[%1] name='%2' class=%3 status=%4 %5")
                                 .arg(static_cast<int>(i))
                                 .arg(QString::fromStdString(layer->getName()))
                                 .arg(layer->className())
                                 .arg(QString::fromStdString(status.toString()))
                                 .arg(extentText);

        if (auto* tileLayer = dynamic_cast<osgEarth::TileLayer*>(layer)) {
            qInfo().noquote() << QStringLiteral("[TerrainDiag] tileLayer name='%1' minL=%2 maxL=%3 dataExtents=%4 noData=%5 minValid=%6 maxValid=%7")
                                     .arg(QString::fromStdString(tileLayer->getName()))
                                     .arg(tileLayer->getMinLevel())
                                     .arg(tileLayer->getMaxLevel())
                                     .arg(static_cast<int>(tileLayer->getDataExtentsSize()))
                                     .arg(tileLayer->getNoDataValue(), 0, 'f', 2)
                                     .arg(tileLayer->getMinValidValue(), 0, 'f', 2)
                                     .arg(tileLayer->getMaxValidValue(), 0, 'f', 2);
        }

        if (dynamic_cast<osgEarth::ElevationLayer*>(layer) != nullptr) {
            ++elevationLayerCount;
        }
    }

    qInfo() << "[TerrainDiag] elevationLayerCount=" << elevationLayerCount;

    osgEarth::ElevationPool* pool = map->getElevationPool();
    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (pool == nullptr || wgs84 == nullptr) {
        qWarning() << "[TerrainDiag] ElevationPool or WGS84 SRS unavailable.";
        return;
    }

    const std::array<std::pair<double, double>, 4> probePoints = {
        std::make_pair(86.5, 33.5),   // 青藏高原
        std::make_pair(103.8, 36.1),  // 中国西北过渡带
        std::make_pair(116.4, 39.9),  // 华北平原
        std::make_pair(121.5, 31.2)   // 华东沿海
    };

    bool anyPoolSampleHasData = false;
    for (const auto& probe : probePoints) {
        osgEarth::GeoPoint point(wgs84, probe.first, probe.second, 0.0, osgEarth::ALTMODE_ABSOLUTE);
        const osgEarth::ElevationSample sample = pool->getSample(point, nullptr);
        anyPoolSampleHasData = anyPoolSampleHasData || sample.hasData();
        qInfo().noquote() << QStringLiteral("[TerrainDiag] sample lon=%1 lat=%2 hasData=%3 elev=%4 res=%5")
                                 .arg(probe.first, 0, 'f', 4)
                                 .arg(probe.second, 0, 'f', 4)
                                 .arg(sample.hasData() ? QStringLiteral("true") : QStringLiteral("false"))
                                 .arg(sample.elevation().getValue(), 0, 'f', 2)
                                 .arg(sample.resolution().getValue(), 0, 'f', 2);
    }

    if (!anyPoolSampleHasData) {
        qWarning() << "[TerrainDiag] ElevationPool has no data at startup samples. Running direct ElevationLayer probe...";
        std::vector<osg::ref_ptr<osgEarth::ElevationLayer>> elevationLayers;
        map->getOpenLayers(elevationLayers);
        for (const auto& elevLayerRef : elevationLayers) {
            osgEarth::ElevationLayer* elevLayer = elevLayerRef.get();
            if (elevLayer == nullptr) {
                continue;
            }

            const osgEarth::Profile* profile = elevLayer->getProfile();
            if (profile == nullptr || profile->getSRS() == nullptr) {
                qWarning().noquote() << QStringLiteral("[TerrainDiag] directProbe layer='%1' missing profile/srs")
                                            .arg(QString::fromStdString(elevLayer->getName()));
                continue;
            }

            const unsigned probeLod = std::min(12u, elevLayer->getMaxLevel());

            for (const auto& probe : probePoints) {
                osgEarth::GeoPoint pWgs84(wgs84, probe.first, probe.second, 0.0, osgEarth::ALTMODE_ABSOLUTE);
                osgEarth::GeoPoint pLayer = pWgs84.transform(profile->getSRS());
                osgEarth::TileKey key = profile->createTileKey(pLayer, probeLod);
                if (!key.valid()) {
                    qInfo().noquote() << QStringLiteral("[TerrainDiag] directProbe layer='%1' lon=%2 lat=%3 key=INVALID")
                                             .arg(QString::fromStdString(elevLayer->getName()))
                                             .arg(probe.first, 0, 'f', 4)
                                             .arg(probe.second, 0, 'f', 4);
                    continue;
                }

                osgEarth::GeoHeightField ghf = elevLayer->createHeightField(key);
                if (!ghf.valid()) {
                    qInfo().noquote() << QStringLiteral("[TerrainDiag] directProbe layer='%1' key=%2 hf=INVALID status=%3")
                                             .arg(QString::fromStdString(elevLayer->getName()))
                                             .arg(QString::fromStdString(key.str()))
                                             .arg(QString::fromStdString(ghf.getStatus().toString()));
                    continue;
                }

                const float kNoDataSentinel = std::numeric_limits<float>::lowest();
                float h = kNoDataSentinel;
                const bool ok = ghf.getElevation(profile->getSRS(), pLayer.x(), pLayer.y(), osgEarth::INTERP_BILINEAR, nullptr, h);
                qInfo().noquote() << QStringLiteral("[TerrainDiag] directProbe layer='%1' key=%2 ok=%3 h=%4 hfMin=%5 hfMax=%6")
                                         .arg(QString::fromStdString(elevLayer->getName()))
                                         .arg(QString::fromStdString(key.str()))
                                         .arg(ok ? QStringLiteral("true") : QStringLiteral("false"))
                                         .arg(h, 0, 'f', 2)
                                         .arg(ghf.getMinHeight(), 0, 'f', 2)
                                         .arg(ghf.getMaxHeight(), 0, 'f', 2);
            }
        }
    }
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
            tex->setMaxAnisotropy(8.0f);
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
        mission::ThreatZone adjustedThreat = m_threats[i];
        double groundMeters = 0.0;
        if (sampleTerrainElevationMeters(adjustedThreat.longitudeDeg, adjustedThreat.latitudeDeg, groundMeters)) {
            adjustedThreat.minAltitudeMeters += groundMeters;
            adjustedThreat.maxAltitudeMeters += groundMeters;
        }

        osg::ref_ptr<osg::MatrixTransform> newNode = buildThreatVisualNode(adjustedThreat);
        if (m_threatVisualNodes[i].valid()) {
            m_threatGroup->removeChild(m_threatVisualNodes[i].get());
        }
        m_threatVisualNodes[i] = newNode;
        m_threatGroup->addChild(newNode.get());
    }

    osgEarth::Registry::shaderGenerator().run(m_overlayGroup.get(), "mission-overlay");
}

bool OsgEarthWidget::sampleTerrainElevationMeters(double lonDeg, double latDeg, double& outMeters) const {
    if (!m_mapNode.valid() || m_mapNode->getMap() == nullptr) {
        return false;
    }
    osgEarth::ElevationPool* pool = m_mapNode->getMap()->getElevationPool();
    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (pool == nullptr || wgs84 == nullptr) {
        return false;
    }

    osgEarth::GeoPoint point(wgs84, lonDeg, latDeg, 0.0, osgEarth::ALTMODE_ABSOLUTE);
    const osgEarth::ElevationSample sample = pool->getSample(point, nullptr);
    if (!sample.hasData()) {
        osgEarth::Map* map = m_mapNode->getMap();
        std::vector<osg::ref_ptr<osgEarth::ElevationLayer>> elevationLayers;
        map->getOpenLayers(elevationLayers);
        for (const auto& elevLayerRef : elevationLayers) {
            osgEarth::ElevationLayer* elevLayer = elevLayerRef.get();
            if (elevLayer == nullptr) {
                continue;
            }

            const osgEarth::Profile* profile = elevLayer->getProfile();
            if (profile == nullptr || profile->getSRS() == nullptr) {
                continue;
            }

            osgEarth::GeoPoint pointInLayer = point.transform(profile->getSRS());
            const unsigned lod = std::min(12u, elevLayer->getMaxLevel());
            osgEarth::TileKey key = profile->createTileKey(pointInLayer, lod);
            if (!key.valid()) {
                continue;
            }

            osgEarth::GeoHeightField ghf = elevLayer->createHeightField(key);
            if (!ghf.valid()) {
                continue;
            }

            const float kNoDataSentinel = std::numeric_limits<float>::lowest();
            float height = kNoDataSentinel;
            if (ghf.getElevation(profile->getSRS(), pointInLayer.x(), pointInLayer.y(), osgEarth::INTERP_BILINEAR, nullptr, height)) {
                if (std::isfinite(height) && height > -1.0e20f) {
                    outMeters = static_cast<double>(height);
                    return true;
                }
            }
        }
        return false;
    }

    outMeters = sample.elevation().getValue();
    return true;
}

double OsgEarthWidget::clampAltitudeAboveTerrain(
    double lonDeg,
    double latDeg,
    double desiredAltMeters,
    double minClearanceMeters) const {
    double groundMeters = 0.0;
    if (!sampleTerrainElevationMeters(lonDeg, latDeg, groundMeters)) {
        return desiredAltMeters;
    }
    return std::max(desiredAltMeters, groundMeters + std::max(0.0, minClearanceMeters));
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

    const std::vector<osgEarth::GeoPoint>& renderRoute = mv.route;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(renderRoute.size());

    osg::ref_ptr<osg::Vec3Array> shadowVertices = new osg::Vec3Array;
    shadowVertices->reserve(renderRoute.size());

    for (std::size_t i = 0; i < renderRoute.size(); ++i) {
        const auto& point = renderRoute[i];
        osg::Vec3d world;
        point.toWorld(world);
        vertices->push_back(world);

        const auto* wgs84 = point.getSRS() != nullptr ? point.getSRS() : osgEarth::SpatialReference::get("wgs84");
        if (wgs84 == nullptr) {
            continue;
        }

        osgEarth::GeoPoint shadowPoint(wgs84, point.x(), point.y(), 0.0, osgEarth::ALTMODE_ABSOLUTE);
        osg::Vec3d shadowWorld;
        shadowPoint.toWorld(shadowWorld);
        shadowVertices->push_back(shadowWorld);
    }

    osg::ref_ptr<osg::Geometry> shadowLine = new osg::Geometry;
    shadowLine->setVertexArray(shadowVertices.get());
    shadowLine->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(shadowVertices->size())));
    osg::ref_ptr<osg::Vec4Array> shadowColors = new osg::Vec4Array;
    shadowColors->push_back(osg::Vec4(0.12f, 0.12f, 0.12f, 0.36f));
    shadowLine->setColorArray(shadowColors.get(), osg::Array::BIND_OVERALL);
    osg::StateSet* shadowState = shadowLine->getOrCreateStateSet();
    shadowState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    shadowState->setMode(GL_BLEND, osg::StateAttribute::ON);
    shadowState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> shadowWidth = new osg::LineWidth(3.0f);
    shadowState->setAttributeAndModes(shadowWidth.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geometry> routeBaseLine = new osg::Geometry;
    routeBaseLine->setVertexArray(vertices.get());
    routeBaseLine->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(vertices->size())));
    osg::ref_ptr<osg::Vec4Array> routeBaseColors = new osg::Vec4Array;
    routeBaseColors->push_back(osg::Vec4(0.36f, 0.37f, 0.40f, 0.84f));
    routeBaseLine->setColorArray(routeBaseColors.get(), osg::Array::BIND_OVERALL);
    osg::StateSet* routeBaseState = routeBaseLine->getOrCreateStateSet();
    routeBaseState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    routeBaseState->setMode(GL_BLEND, osg::StateAttribute::ON);
    routeBaseState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> routeBaseWidth = new osg::LineWidth(4.6f);
    routeBaseState->setAttributeAndModes(routeBaseWidth.get(), osg::StateAttribute::ON);

    mv.routeGeode->addDrawable(shadowLine.get());
    mv.routeGeode->addDrawable(routeBaseLine.get());
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

    const std::vector<osgEarth::GeoPoint>& renderTrail = mv.trail;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(renderTrail.size());

    for (const auto& point : renderTrail) {
        osg::Vec3d world;
        point.toWorld(world);
        vertices->push_back(world);
    }

    osg::ref_ptr<osg::Geometry> glow = new osg::Geometry;
    glow->setVertexArray(vertices.get());
    glow->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(vertices->size())));
    osg::ref_ptr<osg::Vec4Array> glowColors = new osg::Vec4Array;
    glowColors->push_back(osg::Vec4(mv.color.r(), mv.color.g(), mv.color.b(), 0.24f));
    glow->setColorArray(glowColors.get(), osg::Array::BIND_OVERALL);
    osg::StateSet* glowState = glow->getOrCreateStateSet();
    glowState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    glowState->setMode(GL_BLEND, osg::StateAttribute::ON);
    glowState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> glowWidth = new osg::LineWidth(9.2f);
    glowState->setAttributeAndModes(glowWidth.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geometry> line = new osg::Geometry;
    line->setVertexArray(vertices.get());
    line->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(vertices->size())));
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(mv.color.r(), mv.color.g(), mv.color.b(), 0.98f));
    line->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
    osg::StateSet* stateSet = line->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth(6.2f);
    stateSet->setAttributeAndModes(width.get(), osg::StateAttribute::ON);

    mv.trailGeode->addDrawable(glow.get());
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
