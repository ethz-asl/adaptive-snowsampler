#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QtGui>

#include "snowsampler_rviz/edit_button.h"
#include "snowsampler_rviz/pose_widget.h"

using namespace snowsampler_rviz;

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QWidget window;
  window.resize(300, 200);
  window.show();
  window.setWindowTitle(QApplication::translate("toplevel", "Top-level widget"));

  QVBoxLayout* layout = new QVBoxLayout;
  PoseWidget* pose_widget = new PoseWidget("a", &window);
  layout->addWidget(pose_widget);
  EditButton* edit_button = new EditButton("a", &window);
  layout->addWidget(edit_button);

  window.setLayout(layout);

  return app.exec();
}
