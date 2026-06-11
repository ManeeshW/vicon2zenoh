#ifndef VICON_GUI_HPP
#define VICON_GUI_HPP

#include <QMainWindow>
#include <QWidget>
#include <QListWidget>
#include <QTabWidget>
#include <QSplitter>
#include <QSpinBox>
#include <QLabel>
#include <QToolBar>
#include <QTimer>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QVector>
#include <QVector3D>
#include <QMap>
#include <QString>
#include <QColor>
#include <QMutex>

#include <deque>
#include <string>
#include <vector>
#include <mutex>

// ─────────────────────────────────────────────────────────────────────────────
// TopicFrame — parsed payload for any subscribed topic
// ─────────────────────────────────────────────────────────────────────────────
struct TopicFrame {
    double ts       = 0;
    double pos[3]   = {};
    double R[3][3]  = {};
    double vel[3]   = {};
    double omega[3] = {};
    bool   has_vel  = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Plot3DWidget — QPainter 3D trajectory viewer with multiple named tracks
// ─────────────────────────────────────────────────────────────────────────────
class Plot3DWidget : public QWidget {
    Q_OBJECT
public:
    explicit Plot3DWidget(QWidget* parent = nullptr);

    /// Add/update the state for a named track.
    void updateTrack(const QString& name, const QColor& color,
                     const QVector3D& pos, const float R[3][3]);
    void clear();

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    struct Track {
        QColor             color;
        QVector<QVector3D> traj;       // circular trajectory buffer
        QVector3D          cur_pos;
        float              cur_R[3][3] = {};
    };

    static constexpr int kMaxPts = 2000;

    float  az_          = 45.0f;
    float  el_          = 25.0f;
    float  scale_       = 80.0f;
    QPoint last_mouse_;
    bool   dragging_    = false;

    QMap<QString, Track> tracks_;      // insertion-ordered by first appearance
    QVector<QString>     track_order_; // keep stable draw order

    QPointF  project(QVector3D p) const;
    void     drawArrow3D(QPainter& p, QVector3D from, QVector3D to,
                         QColor color, int width = 2);
    void     drawFrame(QPainter& painter, QVector3D origin,
                       QVector3D ax, QVector3D ay, QVector3D az,
                       float len, QColor cx, QColor cy, QColor cz);
};

// ─────────────────────────────────────────────────────────────────────────────
// TsChart — single time-series chart with multiple named QLineSeries
// ─────────────────────────────────────────────────────────────────────────────
class TsChart : public QWidget {
    Q_OBJECT
public:
    explicit TsChart(QWidget* parent = nullptr);

    /// Append a data point; creates series on first call for a new name.
    /// Does NOT rebuild axes — call flush() after all addPoints for this tick.
    void addPoint(const QString& name, double t, double val, const QColor& color);

    /// Rebuild axes range if any new points were added since last flush.
    void flush();

    void clear();
    void setHistorySecs(int secs);
    void setYLabel(const QString& label);

private:
    struct SeriesEntry {
        QLineSeries* series = nullptr;
        QColor       color;
    };

    QChartView* view_     = nullptr;
    QChart*     chart_    = nullptr;
    QValueAxis* axis_x_   = nullptr;
    QValueAxis* axis_y_   = nullptr;

    QMap<QString, SeriesEntry> series_map_;
    int    history_secs_ = 30;
    bool   dirty_        = false;

    void rebuildAxesRange();
};

// ─────────────────────────────────────────────────────────────────────────────
// ViconGUI — main window
// ─────────────────────────────────────────────────────────────────────────────
class ViconGUI : public QMainWindow {
    Q_OBJECT
public:
    explicit ViconGUI(QWidget* parent = nullptr);
    ~ViconGUI() = default;

    /// Thread-safe: post a parsed frame from a Zenoh subscriber thread.
    void postFrame(const std::string& topic, const TopicFrame& frame);

    /// Register topic names (call before show()).
    void setTopics(const std::vector<std::string>& object_keys,
                   const std::string& pose_sync_key,
                   const std::string& rel_gt_state_key);

private slots:
    void onDrainTimer();
    void onHzTimer();
    void onClearClicked();
    void onHistoryChanged(int secs);

private:
    // ── Pending frame queue (thread-safe) ────────────────────────────────────
    struct Pending {
        std::string topic;
        TopicFrame  frame;
    };
    std::deque<Pending> pending_;
    std::mutex          pending_mutex_;

    // ── Topic list ────────────────────────────────────────────────────────────
    QListWidget* topic_list_  = nullptr;

    // ── Tabs ──────────────────────────────────────────────────────────────────
    QTabWidget*  tabs_        = nullptr;

    // Position tab
    Plot3DWidget* plot3d_     = nullptr;
    TsChart*      chart_x_    = nullptr;
    TsChart*      chart_y_    = nullptr;
    TsChart*      chart_z_    = nullptr;

    // Velocity tab
    TsChart*      chart_vx_   = nullptr;
    TsChart*      chart_vy_   = nullptr;
    TsChart*      chart_vz_   = nullptr;

    // Orientation tab
    TsChart*      chart_roll_ = nullptr;
    TsChart*      chart_pitch_= nullptr;
    TsChart*      chart_yaw_  = nullptr;

    // ── Toolbar ───────────────────────────────────────────────────────────────
    QSpinBox*    history_spin_ = nullptr;
    QLabel*      hz_label_     = nullptr;

    // ── Timers ───────────────────────────────────────────────────────────────
    QTimer* drain_timer_ = nullptr;
    QTimer* hz_timer_    = nullptr;

    // ── Hz counter ───────────────────────────────────────────────────────────
    int frame_count_     = 0;

    // ── Topic → colour map ────────────────────────────────────────────────────
    QMap<QString, QColor> topic_colors_;
    QColor colorForTopic(const QString& topic);

    // ── Known topic names ─────────────────────────────────────────────────────
    std::string pose_sync_key_;
    std::string rel_gt_state_key_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    bool isTopicEnabled(const QString& topic) const;
    void buildUI();
    void applyDarkTheme();
};

#endif // VICON_GUI_HPP
