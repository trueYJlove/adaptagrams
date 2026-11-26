#include "library_widget.h"

#include <QDataStream>
#include <QDrag>
#include <QIODevice>
#include <QMimeData>
#include <QPainter>
#include <QPixmap>
#include <vector>

namespace {

constexpr int kRoleTemplateData = Qt::UserRole + 1;

QPixmap RenderTemplateIcon(const BlockTemplate& t) {
  QPixmap pix(72, 48);
  pix.fill(Qt::transparent);
  QPainter p(&pix);
  p.setRenderHint(QPainter::Antialiasing, true);

  const QRectF r(14, 8, 44, 32);
  const bool is_obstacle =
      (t.left_pins + t.right_pins + t.top_pins + t.bottom_pins) == 0;
  p.setPen(QPen(is_obstacle ? QColor(180, 95, 40) : QColor(58, 76, 98), 1.5));
  p.setBrush(is_obstacle ? QColor(255, 238, 224) : QColor(255, 255, 255));
  p.drawRoundedRect(r, 4.0, 4.0);

  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 120, 215));
  for (int i = 0; i < t.left_pins; ++i) {
    const double y = r.top() + (i + 1.0) / (t.left_pins + 1.0) * r.height();
    p.drawEllipse(QPointF(r.left(), y), 2.5, 2.5);
  }
  for (int i = 0; i < t.right_pins; ++i) {
    const double y = r.top() + (i + 1.0) / (t.right_pins + 1.0) * r.height();
    p.drawEllipse(QPointF(r.right(), y), 2.5, 2.5);
  }
  for (int i = 0; i < t.top_pins; ++i) {
    const double x = r.left() + (i + 1.0) / (t.top_pins + 1.0) * r.width();
    p.drawEllipse(QPointF(x, r.top()), 2.5, 2.5);
  }
  for (int i = 0; i < t.bottom_pins; ++i) {
    const double x = r.left() + (i + 1.0) / (t.bottom_pins + 1.0) * r.width();
    p.drawEllipse(QPointF(x, r.bottom()), 2.5, 2.5);
  }
  return pix;
}

}  // namespace

QByteArray EncodeBlockTemplate(const BlockTemplate& tmpl) {
  QByteArray bytes;
  QDataStream out(&bytes, QIODevice::WriteOnly);
  out << tmpl.name << tmpl.width << tmpl.height << tmpl.left_pins
      << tmpl.right_pins << tmpl.top_pins << tmpl.bottom_pins;
  return bytes;
}

bool DecodeBlockTemplate(const QByteArray& data, BlockTemplate* out) {
  if (!out || data.isEmpty()) {
    return false;
  }
  QDataStream in(data);
  in >> out->name >> out->width >> out->height >> out->left_pins >>
      out->right_pins >> out->top_pins >> out->bottom_pins;
  return in.status() == QDataStream::Ok;
}

LibraryWidget::LibraryWidget(QWidget* parent) : QListWidget(parent) {
  setDragEnabled(true);
  setSelectionMode(QAbstractItemView::SingleSelection);
  setIconSize(QSize(72, 48));
  setSpacing(4);
  PopulateTemplates();

  connect(
      this, &QListWidget::itemDoubleClicked, this,
      [this](QListWidgetItem* item) {
        BlockTemplate tmpl;
        if (item && DecodeBlockTemplate(
                        item->data(kRoleTemplateData).toByteArray(), &tmpl)) {
          emit TemplateActivated(tmpl);
        }
      });
}

void LibraryWidget::PopulateTemplates() {
  const std::vector<BlockTemplate> presets = {
      {"基础模块 (1入 / 1出)", 100.0, 60.0, 1, 1, 0, 0},
      {"多入汇聚 (2入 / 1出)", 100.0, 80.0, 2, 1, 0, 0},
      {"信号分路 (1入 / 2出)", 100.0, 80.0, 1, 2, 0, 0},
      {"子系统 (2入 / 2出)", 120.0, 84.0, 2, 2, 0, 0},
      {"四向模块 (上下左右各1)", 100.0, 70.0, 1, 1, 1, 1},
      {"障碍物块 (0端口)", 120.0, 70.0, 0, 0, 0, 0},
      {"紧凑节点 (1入 / 1出)", 64.0, 44.0, 1, 1, 0, 0},
  };
  for (const auto& t : presets) {
    auto* item =
        new QListWidgetItem(QIcon(RenderTemplateIcon(t)), t.name, this);
    item->setData(kRoleTemplateData, EncodeBlockTemplate(t));
    item->setToolTip("拖拽到右侧画布放置，或双击在视口中心创建");
  }
}

void LibraryWidget::startDrag(Qt::DropActions /*supported_actions*/) {
  QListWidgetItem* item = currentItem();
  if (!item) {
    return;
  }
  const QByteArray payload = item->data(kRoleTemplateData).toByteArray();
  auto* mime = new QMimeData();
  mime->setData(kBlockTemplateMime, payload);

  auto* drag = new QDrag(this);
  drag->setMimeData(mime);
  BlockTemplate tmpl;
  if (DecodeBlockTemplate(payload, &tmpl)) {
    const QPixmap icon = RenderTemplateIcon(tmpl);
    drag->setPixmap(icon);
    drag->setHotSpot(QPoint(icon.width() / 2, icon.height() / 2));
  }
  drag->exec(Qt::CopyAction);
}
