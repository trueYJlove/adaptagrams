#ifndef LOCALROUTE_QT_DEMO_LIBRARY_WIDGET_H_
#define LOCALROUTE_QT_DEMO_LIBRARY_WIDGET_H_

#include <QByteArray>
#include <QListWidget>
#include <QString>

struct BlockTemplate {
  QString name;
  double width = 100.0;
  double height = 60.0;
  int left_pins = 1;
  int right_pins = 1;
  int top_pins = 0;
  int bottom_pins = 0;
};

inline constexpr char kBlockTemplateMime[] = "application/x-localroute-block";

QByteArray EncodeBlockTemplate(const BlockTemplate& tmpl);
bool DecodeBlockTemplate(const QByteArray& data, BlockTemplate* out);

class LibraryWidget : public QListWidget {
  Q_OBJECT

 public:
  explicit LibraryWidget(QWidget* parent = nullptr);

 signals:
  void TemplateActivated(const BlockTemplate& tmpl);

 protected:
  void startDrag(Qt::DropActions supported_actions) override;

 private:
  void PopulateTemplates();
};

#endif  // LOCALROUTE_QT_DEMO_LIBRARY_WIDGET_H_
