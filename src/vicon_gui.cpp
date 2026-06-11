#include "vicon_gui.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QApplication>
#include <QToolBar>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QListWidgetItem>
#include <QtCharts/QChart>
#include <QFont>
#include <QBrush>
#include <QPen>
#include <cmath>
#include <algorithm>
#include <unordered_map>

// ─────────────────────────────────────────────────────────────────────────────
// Dark theme palette (VSCode Dark+)
// ─────────────────────────────────────────────────────────────────────────────
static const QColor kWinBg   ("#1e1e1e");
static const QColor kPanelBg ("#252526");
static const QColor kBorder  ("#474747");
static const QColor kText    ("#d4d4d4");
static const QColor kAccent  ("#007acc");
static const QColor kGrid    ("#3e3e42");
static const QColor kPlotBg  ("#1b1b1b");

// Topic colours
static const QColor kColorPoseSync   ("#c586c0");   // purple
static const QColor kColorRelGtState ("#4fc1ff");   // cyan-blue
static const QColor kColorObjects[]  = {
    QColor("#f14c4c"),  // red     – object[0]
    QColor("#4ec994"),  // green   – object[1]
    QColor("#ffab40"),  // amber   – object[2]
    QColor("#e040fb"),  // violet  – object[3]
    QColor("#89d9a8"),  // mint    – object[4]
    QColor("#ff6d00"),  // orange  – object[5]
};
static constexpr int kNumObjectColors =
    static_cast<int>(sizeof(kColorObjects) / sizeof(kColorObjects[0]));


// ═══════════════════════════════════════════════════════════════════════════════
// Plot3DWidget
// ═══════════════════════════════════════════════════════════════════════════════

Plot3DWidget::Plot3DWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(300, 300);
    setMouseTracking(true);
}

void Plot3DWidget::updateTrack(const QString& name, const QColor& color,
                                const QVector3D& pos, const float R[3][3]) {
    if (!tracks_.contains(name)) {
        Track t;
        t.color = color;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                t.cur_R[i][j] = (i == j) ? 1.0f : 0.0f;
        tracks_.insert(name, t);
        track_order_.append(name);
    }
    Track& tr = tracks_[name];
    tr.color   = color;
    tr.cur_pos = pos;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            tr.cur_R[i][j] = R[i][j];

    if (tr.traj.size() >= kMaxPts)
        tr.traj.removeFirst();
    tr.traj.append(pos);

    update();
}

void Plot3DWidget::clear() {
    for (auto& tr : tracks_)
        tr.traj.clear();
    update();
}

// ── Projection ────────────────────────────────────────────────────────────────
QPointF Plot3DWidget::project(QVector3D p) const {
    float az = az_ * float(M_PI) / 180.0f;
    float el = el_ * float(M_PI) / 180.0f;

    float x1 =  p.x() * std::cos(az) + p.y() * std::sin(az);
    float y1 = -p.x() * std::sin(az) + p.y() * std::cos(az);
    float z1 = -p.z();

    float sx = x1;
    float sy = z1 * std::cos(el) - y1 * std::sin(el);

    return QPointF(width()  / 2.0 + sx * scale_,
                   height() / 2.0 - sy * scale_);
}

// ── Arrow drawing ─────────────────────────────────────────────────────────────
void Plot3DWidget::drawArrow3D(QPainter& p, QVector3D from, QVector3D to,
                                QColor color, int w) {
    QPointF pf = project(from);
    QPointF pt = project(to);
    p.setPen(QPen(color, w, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(pf, pt);

    QLineF line(pf, pt);
    if (line.length() < 2) return;
    double angle = std::atan2(pt.y() - pf.y(), pt.x() - pf.x());
    const double sz = 8.0;
    QPointF t1 = pt + QPointF(std::cos(angle - 2.4) * sz, std::sin(angle - 2.4) * sz);
    QPointF t2 = pt + QPointF(std::cos(angle + 2.4) * sz, std::sin(angle + 2.4) * sz);
    QPolygonF head;
    head << pt << t1 << t2;
    p.setBrush(color);
    p.setPen(Qt::NoPen);
    p.drawPolygon(head);
    p.setBrush(Qt::NoBrush);
}

void Plot3DWidget::drawFrame(QPainter& painter, QVector3D origin,
                              QVector3D ax, QVector3D ay, QVector3D az,
                              float len, QColor cx, QColor cy, QColor cz) {
    drawArrow3D(painter, origin, origin + ax * len, cx, 2);
    drawArrow3D(painter, origin, origin + ay * len, cy, 2);
    drawArrow3D(painter, origin, origin + az * len, cz, 2);
}

// ── Paint ─────────────────────────────────────────────────────────────────────
void Plot3DWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    p.fillRect(rect(), kPanelBg);

    // XY grid at z=0
    p.setPen(QPen(kGrid, 1, Qt::DotLine));
    for (int i = -5; i <= 5; ++i) {
        p.drawLine(project({float(i), -5.0f, 0.0f}), project({float(i),  5.0f, 0.0f}));
        p.drawLine(project({-5.0f, float(i), 0.0f}), project({ 5.0f, float(i), 0.0f}));
    }

    // World frame axes at origin
    {
        const float len = 0.5f;
        drawArrow3D(p, {0,0,0}, {len,0,0}, QColor("#f14c4c"), 2);
        drawArrow3D(p, {0,0,0}, {0,len,0}, QColor("#4ec994"), 2);
        drawArrow3D(p, {0,0,0}, {0,0,len}, QColor("#4fc1ff"), 2);
        p.setPen(kText);
        QFont f = p.font(); f.setPointSize(8); p.setFont(f);
        p.drawText(project({len + 0.05f, 0, 0}), "X");
        p.drawText(project({0, len + 0.05f, 0}), "Y");
        p.drawText(project({0, 0, len + 0.05f}), "Z");
    }

    // Per-track trajectories, dots, and body frames
    for (const QString& name : track_order_) {
        const Track& tr = tracks_[name];
        if (tr.traj.isEmpty()) continue;

        const int n = tr.traj.size();

        // Gradient trajectory old→bright
        QColor old_c = tr.color.darker(300);
        for (int i = 1; i < n; ++i) {
            float t = float(i) / float(n - 1);
            QColor c(
                int(old_c.red()   + t * (tr.color.red()   - old_c.red())),
                int(old_c.green() + t * (tr.color.green() - old_c.green())),
                int(old_c.blue()  + t * (tr.color.blue()  - old_c.blue()))
            );
            p.setPen(QPen(c, 2, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(project(tr.traj[i-1]), project(tr.traj[i]));
        }

        // Current position dot
        QPointF cp = project(tr.cur_pos);
        p.setPen(Qt::NoPen);
        p.setBrush(tr.color);
        p.drawEllipse(cp, 5, 5);
        p.setBrush(Qt::NoBrush);

        // Body frame axes (columns of R)
        const float len = 0.30f;
        QVector3D bx(tr.cur_R[0][0], tr.cur_R[1][0], tr.cur_R[2][0]);
        QVector3D by(tr.cur_R[0][1], tr.cur_R[1][1], tr.cur_R[2][1]);
        QVector3D bz(tr.cur_R[0][2], tr.cur_R[1][2], tr.cur_R[2][2]);
        // Slightly lighten the frame colours relative to traj colour
        QColor cx = tr.color.lighter(130);
        QColor cy = tr.color.lighter(110);
        QColor cz = tr.color.lighter(150);
        drawArrow3D(p, tr.cur_pos, tr.cur_pos + bx * len, cx, 2);
        drawArrow3D(p, tr.cur_pos, tr.cur_pos + by * len, cy, 2);
        drawArrow3D(p, tr.cur_pos, tr.cur_pos + bz * len, cz, 2);
    }

    // Corner info
    p.setPen(kText);
    QFont fi = p.font(); fi.setPointSize(9); p.setFont(fi);
    p.drawText(8, 20, QString("az: %1°  el: %2°  scale: %3 px/m")
               .arg(az_, 0, 'f', 1)
               .arg(el_, 0, 'f', 1)
               .arg(scale_, 0, 'f', 0));

    int line = 38;
    for (const QString& name : track_order_) {
        const Track& tr = tracks_[name];
        if (tr.traj.isEmpty()) continue;
        p.setPen(tr.color);
        p.drawText(8, line,
                   QString("%1: [%2, %3, %4] m")
                   .arg(name)
                   .arg(tr.cur_pos.x(), 0, 'f', 3)
                   .arg(tr.cur_pos.y(), 0, 'f', 3)
                   .arg(tr.cur_pos.z(), 0, 'f', 3));
        line += 18;
    }
}

// ── Mouse interaction ─────────────────────────────────────────────────────────
void Plot3DWidget::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        last_mouse_ = e->pos();
        dragging_   = true;
    }
}
void Plot3DWidget::mouseReleaseEvent(QMouseEvent*) { dragging_ = false; }
void Plot3DWidget::mouseMoveEvent(QMouseEvent* e) {
    if (!dragging_) return;
    QPoint d = e->pos() - last_mouse_;
    az_ += d.x() * 0.5f;
    el_  = std::clamp(el_ - d.y() * 0.5f, -89.0f, 89.0f);
    last_mouse_ = e->pos();
    update();
}
void Plot3DWidget::wheelEvent(QWheelEvent* e) {
    float factor = (e->angleDelta().y() > 0) ? 1.15f : 0.87f;
    scale_ = std::clamp(scale_ * factor, 10.0f, 2000.0f);
    update();
}


// ═══════════════════════════════════════════════════════════════════════════════
// TsChart
// ═══════════════════════════════════════════════════════════════════════════════

TsChart::TsChart(QWidget* parent) : QWidget(parent) {
    chart_  = new QChart();
    axis_x_ = new QValueAxis();
    axis_y_ = new QValueAxis();

    chart_->setBackgroundBrush(QBrush(kPanelBg));
    chart_->setPlotAreaBackgroundBrush(QBrush(kPlotBg));
    chart_->setPlotAreaBackgroundVisible(true);
    chart_->setMargins({4, 4, 4, 4});
    chart_->setContentsMargins(0, 0, 0, 0);

    auto styleAxis = [&](QValueAxis* ax, const QString& label) {
        ax->setLabelsBrush(QBrush(kText));
        ax->setTitleBrush(QBrush(kText));
        ax->setTitleText(label);
        QFont af; af.setPointSize(8);
        ax->setLabelsFont(af);
        ax->setTitleFont(af);
        ax->setGridLineColor(kGrid);
        ax->setLinePen(QPen(kGrid));
        ax->setMinorGridLineColor(QColor("#232323"));
    };
    styleAxis(axis_x_, "t (s)");
    styleAxis(axis_y_, "value");

    chart_->addAxis(axis_x_, Qt::AlignBottom);
    chart_->addAxis(axis_y_, Qt::AlignLeft);

    // Legend — right side, dark styling
    chart_->legend()->setVisible(true);
    chart_->legend()->setAlignment(Qt::AlignRight);
    chart_->legend()->setLabelColor(kText);
    chart_->legend()->setBackgroundVisible(true);
    chart_->legend()->setBrush(QBrush(kPanelBg));
    chart_->legend()->setPen(QPen(kBorder));

    view_ = new QChartView(chart_, this);
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setStyleSheet("background: transparent; border: none;");

    auto* lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(view_);
}

void TsChart::addPoint(const QString& name, double t, double val,
                        const QColor& color) {
    if (!series_map_.contains(name)) {
        auto* s = new QLineSeries();
        s->setName(name);
        s->setColor(color);
        s->setPen(QPen(color, 1.5));
        chart_->addSeries(s);
        s->attachAxis(axis_x_);
        s->attachAxis(axis_y_);
        series_map_.insert(name, {s, color});
    }

    QLineSeries* s = series_map_[name].series;
    s->append(t, val);

    // Trim old points outside rolling window
    while (s->count() > 1 && t - s->at(0).x() > history_secs_)
        s->remove(0);

    dirty_ = true;
}

void TsChart::flush() {
    if (!dirty_) return;
    dirty_ = false;
    rebuildAxesRange();
}

void TsChart::clear() {
    for (auto& entry : series_map_)
        entry.series->clear();
    axis_x_->setRange(0, history_secs_);
    axis_y_->setRange(-1, 1);
}

void TsChart::setHistorySecs(int secs) {
    history_secs_ = secs;
}

void TsChart::setYLabel(const QString& label) {
    axis_y_->setTitleText(label);
}

void TsChart::rebuildAxesRange() {
    double t_min = 1e18, t_max = -1e18;
    double y_min = 1e18, y_max = -1e18;

    for (const auto& entry : series_map_) {
        const QLineSeries* s = entry.series;
        for (const QPointF& pt : s->points()) {
            t_min = std::min(t_min, pt.x());
            t_max = std::max(t_max, pt.x());
            y_min = std::min(y_min, pt.y());
            y_max = std::max(y_max, pt.y());
        }
    }

    if (t_min > t_max) {
        axis_x_->setRange(0, history_secs_);
        axis_y_->setRange(-1, 1);
        return;
    }

    axis_x_->setRange(t_min, std::max(t_min + 1.0, t_max));

    double margin = std::max(0.05, (y_max - y_min) * 0.15);
    axis_y_->setRange(y_min - margin, y_max + margin);
}


// ═══════════════════════════════════════════════════════════════════════════════
// ViconGUI
// ═══════════════════════════════════════════════════════════════════════════════

ViconGUI::ViconGUI(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("Vicon2Zenoh Live Monitor");
    applyDarkTheme();
    buildUI();

    drain_timer_ = new QTimer(this);
    drain_timer_->setInterval(33); // ~30 Hz
    connect(drain_timer_, &QTimer::timeout, this, &ViconGUI::onDrainTimer);
    drain_timer_->start();

    hz_timer_ = new QTimer(this);
    hz_timer_->setInterval(1000);
    connect(hz_timer_, &QTimer::timeout, this, &ViconGUI::onHzTimer);
    hz_timer_->start();
}

void ViconGUI::setTopics(const std::vector<std::string>& object_keys,
                          const std::string& pose_sync_key,
                          const std::string& rel_gt_state_key) {
    pose_sync_key_    = pose_sync_key;
    rel_gt_state_key_ = rel_gt_state_key;

    topic_list_->clear();

    // Helper to add a checkbox item
    auto addItem = [&](const QString& name, bool checked) {
        auto* item = new QListWidgetItem(name, topic_list_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
        item->setForeground(colorForTopic(name));
    };

    // Objects — unchecked by default
    for (const auto& k : object_keys)
        addItem(QString::fromStdString(k), false);

    // Pose sync & rel_gt_state — checked by default
    if (!pose_sync_key.empty())
        addItem(QString::fromStdString(pose_sync_key), true);
    if (!rel_gt_state_key.empty())
        addItem(QString::fromStdString(rel_gt_state_key), true);
}

// ── postFrame ─────────────────────────────────────────────────────────────────
void ViconGUI::postFrame(const std::string& topic, const TopicFrame& frame) {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    pending_.push_back({topic, frame});
    // Bound the queue to avoid memory runaway if drain stalls
    while (pending_.size() > 2000)
        pending_.pop_front();
}

// ── Drain timer (30 Hz) ───────────────────────────────────────────────────────
void ViconGUI::onDrainTimer() {
    std::deque<Pending> local;
    {
        std::lock_guard<std::mutex> lk(pending_mutex_);
        std::swap(local, pending_);
    }

    // Keep only the latest frame per topic to avoid Qt chart work piling up
    std::unordered_map<std::string, const Pending*> latest;
    for (const Pending& p : local)
        latest[p.topic] = &p;

    for (auto& [topic_str, pend] : latest) {
        const QString  qtopic = QString::fromStdString(topic_str);
        const TopicFrame& f  = pend->frame;

        if (!isTopicEnabled(qtopic)) continue;

        ++frame_count_;

        const QColor color = colorForTopic(qtopic);

        // ZYX Euler angles  R = Rz(yaw)*Ry(pitch)*Rx(roll)
        const double (*R)[3] = f.R;
        double roll  = std::atan2( R[2][1],  R[2][2]) * 180.0 / M_PI;
        double pitch = std::asin(std::clamp(-R[2][0], -1.0, 1.0)) * 180.0 / M_PI;
        double yaw   = std::atan2( R[1][0],  R[0][0]) * 180.0 / M_PI;

        const double t = f.ts;

        chart_x_->addPoint(qtopic, t, f.pos[0], color);
        chart_y_->addPoint(qtopic, t, f.pos[1], color);
        chart_z_->addPoint(qtopic, t, f.pos[2], color);

        chart_roll_->addPoint(qtopic,  t, roll,  color);
        chart_pitch_->addPoint(qtopic, t, pitch, color);
        chart_yaw_->addPoint(qtopic,   t, yaw,   color);

        float Rf[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Rf[i][j] = float(R[i][j]);
        plot3d_->updateTrack(qtopic, color,
                             QVector3D(float(f.pos[0]),
                                       float(f.pos[1]),
                                       float(f.pos[2])),
                             Rf);

        if (f.has_vel) {
            chart_vx_->addPoint(qtopic, t, f.vel[0], color);
            chart_vy_->addPoint(qtopic, t, f.vel[1], color);
            chart_vz_->addPoint(qtopic, t, f.vel[2], color);
        }
    }

    // Rebuild axes once per drain cycle (not per addPoint)
    chart_x_->flush();    chart_y_->flush();     chart_z_->flush();
    chart_vx_->flush();   chart_vy_->flush();    chart_vz_->flush();
    chart_roll_->flush(); chart_pitch_->flush(); chart_yaw_->flush();
}

// ── Hz timer ──────────────────────────────────────────────────────────────────
void ViconGUI::onHzTimer() {
    hz_label_->setText(QString("  %1 Hz  ").arg(frame_count_));
    frame_count_ = 0;
}

// ── Clear ──────────────────────────────────────────────────────────────────────
void ViconGUI::onClearClicked() {
    plot3d_->clear();
    chart_x_->clear();    chart_y_->clear();     chart_z_->clear();
    chart_vx_->clear();   chart_vy_->clear();    chart_vz_->clear();
    chart_roll_->clear(); chart_pitch_->clear(); chart_yaw_->clear();
}

// ── History changed ───────────────────────────────────────────────────────────
void ViconGUI::onHistoryChanged(int secs) {
    chart_x_->setHistorySecs(secs);     chart_y_->setHistorySecs(secs);
    chart_z_->setHistorySecs(secs);     chart_vx_->setHistorySecs(secs);
    chart_vy_->setHistorySecs(secs);    chart_vz_->setHistorySecs(secs);
    chart_roll_->setHistorySecs(secs);  chart_pitch_->setHistorySecs(secs);
    chart_yaw_->setHistorySecs(secs);
}

// ── buildUI ───────────────────────────────────────────────────────────────────
void ViconGUI::buildUI() {
    resize(1400, 800);

    // ── Toolbar ───────────────────────────────────────────────────────────────
    auto* toolbar = addToolBar("Main");
    toolbar->setMovable(false);
    toolbar->setFloatable(false);
    toolbar->setStyleSheet(
        "QToolBar { background: #252526; border-bottom: 1px solid #474747; spacing: 6px; }"
        "QToolBar QLabel { color: #d4d4d4; }"
    );

    toolbar->addWidget(new QLabel("  Objects:"));

    toolbar->addSeparator();

    toolbar->addWidget(new QLabel("  History:"));
    history_spin_ = new QSpinBox();
    history_spin_->setRange(5, 300);
    history_spin_->setValue(30);
    history_spin_->setSuffix(" s");
    history_spin_->setFixedWidth(70);
    history_spin_->setStyleSheet(
        "QSpinBox { background: #3c3c3c; color: #d4d4d4; border: 1px solid #474747; "
        "           border-radius: 3px; padding: 2px; }"
    );
    toolbar->addWidget(history_spin_);

    auto* clear_btn = new QPushButton("Clear");
    clear_btn->setFixedWidth(55);
    clear_btn->setStyleSheet(
        "QPushButton { background: #3c3c3c; color: #d4d4d4; border: 1px solid #474747; "
        "              border-radius: 3px; padding: 3px 8px; }"
        "QPushButton:hover { background: #505050; }"
        "QPushButton:pressed { background: #007acc; }"
    );
    toolbar->addWidget(clear_btn);

    toolbar->addSeparator();

    hz_label_ = new QLabel("  -- Hz  ");
    hz_label_->setStyleSheet("color: #4fc1ff; font-weight: bold;");
    toolbar->addWidget(hz_label_);

    connect(clear_btn,    &QPushButton::clicked,
            this,         &ViconGUI::onClearClicked);
    connect(history_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
            this,          &ViconGUI::onHistoryChanged);

    // ── Central widget ────────────────────────────────────────────────────────
    auto* central     = new QWidget(this);
    auto* main_layout = new QHBoxLayout(central);
    main_layout->setContentsMargins(4, 4, 4, 4);
    main_layout->setSpacing(4);
    setCentralWidget(central);

    // ── Left panel — topic list ───────────────────────────────────────────────
    auto* left_panel = new QWidget();
    left_panel->setFixedWidth(180);
    left_panel->setStyleSheet(
        "QWidget { background: #252526; border-right: 1px solid #474747; }"
    );
    auto* left_lay = new QVBoxLayout(left_panel);
    left_lay->setContentsMargins(4, 6, 4, 4);
    left_lay->setSpacing(4);

    auto* topics_label = new QLabel("Topics");
    topics_label->setStyleSheet("color: #007acc; font-weight: bold; font-size: 11px;");
    left_lay->addWidget(topics_label);

    topic_list_ = new QListWidget();
    topic_list_->setStyleSheet(
        "QListWidget { background: #1e1e1e; border: 1px solid #474747; color: #d4d4d4; }"
        "QListWidget::item { padding: 3px; }"
        "QListWidget::item:selected { background: #094771; }"
    );
    left_lay->addWidget(topic_list_);
    main_layout->addWidget(left_panel);

    // ── Right: tabs ───────────────────────────────────────────────────────────
    tabs_ = new QTabWidget();
    tabs_->setStyleSheet(
        "QTabWidget::pane { border: 1px solid #474747; background: #1e1e1e; }"
        "QTabBar::tab { background: #2d2d30; color: #d4d4d4; "
        "               padding: 5px 12px; border: 1px solid #474747; "
        "               border-bottom: none; margin-right: 2px; }"
        "QTabBar::tab:selected { background: #1e1e1e; color: #ffffff; }"
        "QTabBar::tab:hover { background: #3e3e42; }"
    );
    main_layout->addWidget(tabs_, 1);

    // ── Tab: Position ─────────────────────────────────────────────────────────
    {
        auto* tab = new QWidget();
        auto* lay = new QHBoxLayout(tab);
        lay->setContentsMargins(2, 2, 2, 2);
        lay->setSpacing(4);

        auto* splitter = new QSplitter(Qt::Horizontal);

        plot3d_ = new Plot3DWidget();
        splitter->addWidget(plot3d_);

        auto* ts_widget = new QWidget();
        auto* ts_lay    = new QVBoxLayout(ts_widget);
        ts_lay->setContentsMargins(0, 0, 0, 0);
        ts_lay->setSpacing(2);

        chart_x_ = new TsChart();  chart_x_->setYLabel("X (m)");
        chart_y_ = new TsChart();  chart_y_->setYLabel("Y (m)");
        chart_z_ = new TsChart();  chart_z_->setYLabel("Z (m)");
        ts_lay->addWidget(chart_x_);
        ts_lay->addWidget(chart_y_);
        ts_lay->addWidget(chart_z_);
        splitter->addWidget(ts_widget);

        splitter->setStretchFactor(0, 1);
        splitter->setStretchFactor(1, 1);
        splitter->setStyleSheet("QSplitter::handle { background: #474747; width: 3px; }");

        lay->addWidget(splitter);
        tabs_->addTab(tab, "Position");
    }

    // ── Tab: Velocity ─────────────────────────────────────────────────────────
    {
        auto* tab = new QWidget();
        auto* lay = new QVBoxLayout(tab);
        lay->setContentsMargins(4, 4, 4, 4);
        lay->setSpacing(2);

        chart_vx_ = new TsChart();  chart_vx_->setYLabel("Vx (m/s)");
        chart_vy_ = new TsChart();  chart_vy_->setYLabel("Vy (m/s)");
        chart_vz_ = new TsChart();  chart_vz_->setYLabel("Vz (m/s)");
        lay->addWidget(chart_vx_);
        lay->addWidget(chart_vy_);
        lay->addWidget(chart_vz_);
        tabs_->addTab(tab, "Velocity");
    }

    // ── Tab: Orientation ─────────────────────────────────────────────────────
    {
        auto* tab = new QWidget();
        auto* lay = new QVBoxLayout(tab);
        lay->setContentsMargins(4, 4, 4, 4);
        lay->setSpacing(2);

        chart_roll_  = new TsChart();  chart_roll_->setYLabel("Roll (°)");
        chart_pitch_ = new TsChart();  chart_pitch_->setYLabel("Pitch (°)");
        chart_yaw_   = new TsChart();  chart_yaw_->setYLabel("Yaw (°)");
        lay->addWidget(chart_roll_);
        lay->addWidget(chart_pitch_);
        lay->addWidget(chart_yaw_);
        tabs_->addTab(tab, "Orientation");
    }
}

// ── applyDarkTheme ────────────────────────────────────────────────────────────
void ViconGUI::applyDarkTheme() {
    setStyleSheet(
        "QMainWindow, QWidget {"
        "    background-color: #1e1e1e;"
        "    color: #d4d4d4;"
        "}"
        "QScrollBar:vertical {"
        "    background: #252526; width: 10px; margin: 0;"
        "}"
        "QScrollBar::handle:vertical {"
        "    background: #474747; min-height: 20px; border-radius: 4px;"
        "}"
        "QScrollBar:horizontal {"
        "    background: #252526; height: 10px; margin: 0;"
        "}"
        "QScrollBar::handle:horizontal {"
        "    background: #474747; min-width: 20px; border-radius: 4px;"
        "}"
        "QSplitter::handle { background: #474747; }"
    );
}

// ── colorForTopic ─────────────────────────────────────────────────────────────
QColor ViconGUI::colorForTopic(const QString& topic) {
    if (topic_colors_.contains(topic))
        return topic_colors_[topic];

    QColor c;
    const std::string st = topic.toStdString();
    if (st == pose_sync_key_) {
        c = kColorPoseSync;
    } else if (st == rel_gt_state_key_) {
        c = kColorRelGtState;
    } else {
        // Object colours assigned in order of first appearance
        int obj_idx = 0;
        for (const QString& k : topic_colors_.keys()) {
            const std::string sk = k.toStdString();
            if (sk != pose_sync_key_ && sk != rel_gt_state_key_)
                ++obj_idx;
        }
        c = kColorObjects[obj_idx % kNumObjectColors];
    }

    topic_colors_.insert(topic, c);
    return c;
}

// ── isTopicEnabled ────────────────────────────────────────────────────────────
bool ViconGUI::isTopicEnabled(const QString& topic) const {
    for (int i = 0; i < topic_list_->count(); ++i) {
        QListWidgetItem* item = topic_list_->item(i);
        if (item->text() == topic)
            return item->checkState() == Qt::Checked;
    }
    // Topic not in list → accept it (auto-added topics or wildcard subscribers)
    return true;
}
