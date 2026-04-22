#pragma once

#include <QString>
#include <QWidget>

#include <vector>

#include <osg/ref_ptr>

#include <osgEarth/GeoData>

namespace osg {
class Geode;
class Group;
class MatrixTransform;
class Node;
}

namespace osgViewer {
class GraphicsWindow;
class Viewer;
}

namespace osgGA {
class TrackballManipulator;
}

namespace osgEarth {
class MapNode;
class GeoTransform;
namespace Util {
class EarthManipulator;
}
}

namespace mission {
struct ThreatZone;
}

class OsgEarthWidget : public QWidget {
    Q_OBJECT

public:
    enum class GlobeMode {
        Realistic,
        Presentation
    };

    explicit OsgEarthWidget(QWidget* parent = nullptr);
    ~OsgEarthWidget() override;

    void setGlobeMode(GlobeMode mode);
    GlobeMode globeMode() const;
    bool hasRealEarthDataset();
    QString realEarthStatusText() const;

    void setStartPoint(const osgEarth::GeoPoint& point);
    void setTargetPoint(const osgEarth::GeoPoint& point);
    void setThreatZones(const std::vector<mission::ThreatZone>& threats);
    void setPlannedRoute(const std::vector<osgEarth::GeoPoint>& route);
    void setMissilePosition(const osgEarth::GeoPoint& position);
    void setFollowMissile(bool enabled);
    void focusOnPoint(const osgEarth::GeoPoint& point, double rangeMeters);
    void focusOnRoute(const std::vector<osgEarth::GeoPoint>& route);
    void showImpactEffect(const osgEarth::GeoPoint& point);
    void clearImpactEffect();
    void clearMissile();
    void resetMissionGraphics();
    void zoomIn();
    void zoomOut();

protected:
    QPaintEngine* paintEngine() const override;
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private:
    void initializeViewer();
    void syncGraphicsWindowSize();
    void zoomBySteps(int steps);
    void ensureSceneCreated();
    void applyGlobeMode();
    void updateCameraManipulator();
    void rebuildFallbackGlobe();
    void rebuildThreatGeometry();
    void rebuildMarkers();
    void rebuildRouteGeometry();
    void rebuildTrailGeometry();

    bool m_initialized = false;
    bool m_initializing = false;

    osg::ref_ptr<osgViewer::Viewer> m_viewer;
    osg::ref_ptr<osgViewer::GraphicsWindow> m_graphicsWindow;
    osg::ref_ptr<osgEarth::Util::EarthManipulator> m_earthManipulator;
    osg::ref_ptr<osgGA::TrackballManipulator> m_trackballManipulator;

    int m_cachedViewportWidth = 0;
    int m_cachedViewportHeight = 0;

    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<osg::Group> m_realEarthGroup;
    osg::ref_ptr<osgEarth::MapNode> m_mapNode;
    osg::ref_ptr<osg::Group> m_overlayGroup;
    osg::ref_ptr<osg::Group> m_fallbackGroup;
    osg::ref_ptr<osg::Group> m_threatGroup;
    osg::ref_ptr<osg::Group> m_markerGroup;
    osg::ref_ptr<osg::Geode> m_pathGeode;
    osg::ref_ptr<osg::Geode> m_trailGeode;
    osg::ref_ptr<osg::MatrixTransform> m_missileNode;
    osg::ref_ptr<osg::Node> m_impactNode;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> m_threatVisualNodes;

    osgEarth::GeoPoint m_startPoint;
    osgEarth::GeoPoint m_targetPoint;
    osgEarth::GeoPoint m_missilePoint;

    bool m_hasStartPoint = false;
    bool m_hasTargetPoint = false;
    bool m_hasMissilePoint = false;
    bool m_followMissile = false;
    bool m_hasRealEarthDataset = false;
    bool m_realEarthFromLocalFile = false;
    bool m_sceneCreationAttempted = false;
    QString m_realEarthSourcePath;
    GlobeMode m_globeMode = GlobeMode::Realistic;
    int m_followTickCounter = 0;

    std::vector<mission::ThreatZone> m_threats;
    std::vector<osgEarth::GeoPoint> m_route;
    std::vector<osgEarth::GeoPoint> m_trail;
};
