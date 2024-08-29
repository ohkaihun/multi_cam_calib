#include "ui/widgets/charuco_board_target_widget.h"

namespace calibmar {

  CharucoBoardTargetOptionsWidget::CharucoBoardTargetOptionsWidget(QWidget* parent) : QWidget(parent) {
    QLabel* aruco_type_label = new QLabel("Charuco Marker Type", this);
    aruco_type_edit_ = new QComboBox(this);
    for (const auto& name_type : calibration_targets::ArucoTypes()) {
      aruco_type_edit_->addItem(QString::fromStdString(name_type.first));
    }

    QLabel* columns_label = new QLabel(this);
    columns_label->setText("Number of Marker Columns");
    columns_edit_ = new QSpinBox(this);
    columns_edit_->setRange(4, 100);
    columns_edit_->setSingleStep(1);

    QLabel* rows_label = new QLabel(this);
    rows_label->setText("Number of Marker Rows");
    rows_edit_ = new QSpinBox(this);
    rows_edit_->setRange(4, 100);
    rows_edit_->setSingleStep(1);


    QLabel* marker_num_label = new QLabel(this);
    marker_num_label->setText("Number of Markernum");
    marker_num_edit_ = new QSpinBox(this);
    marker_num_edit_->setRange(1, 100);
    marker_num_edit_->setSingleStep(1);


    QLabel* board_index_label = new QLabel(this);
    board_index_label->setText("index of board");
    board_index_edit_ = new QSpinBox(this);
    board_index_edit_->setRange(0, 25);
    board_index_edit_->setSingleStep(1);

    QLabel* marker_size_label = new QLabel(this);
    marker_size_label->setText("Marker Size");
    marker_size_edit_ = new QDoubleSpinBox(this);
    marker_size_edit_->setDecimals(3);
    marker_size_edit_->setRange(0.001, 1000);
    marker_size_edit_->setSingleStep(0.001);
    marker_size_edit_->setValue(1.0);

    QLabel* square_size_label = new QLabel(this);
    square_size_label->setText("Square Size");
    square_size_edit_ = new QDoubleSpinBox(this);
    square_size_edit_->setDecimals(3);
    square_size_edit_->setRange(0.001, 1000);
    square_size_edit_->setSingleStep(0.001);
    square_size_edit_->setValue(1.0);

    QFormLayout* formLayout = new QFormLayout(this);
    formLayout->setWidget(1, QFormLayout::LabelRole, aruco_type_label);
    formLayout->setWidget(1, QFormLayout::FieldRole, aruco_type_edit_);
    formLayout->setWidget(2, QFormLayout::LabelRole, columns_label);
    formLayout->setWidget(2, QFormLayout::FieldRole, columns_edit_);
    formLayout->setWidget(3, QFormLayout::LabelRole, rows_label);
    formLayout->setWidget(3, QFormLayout::FieldRole, rows_edit_);
    formLayout->setWidget(4, QFormLayout::LabelRole, marker_size_label);
    formLayout->setWidget(4, QFormLayout::FieldRole, marker_size_edit_);
    formLayout->setWidget(5, QFormLayout::LabelRole, square_size_label);
    formLayout->setWidget(5, QFormLayout::FieldRole, square_size_edit_);
    formLayout->setWidget(6, QFormLayout::LabelRole, marker_num_label);
    formLayout->setWidget(6, QFormLayout::FieldRole, marker_num_edit_);
    formLayout->setWidget(7, QFormLayout::LabelRole, board_index_label);
    formLayout->setWidget(7, QFormLayout::FieldRole, board_index_edit_);




    formLayout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
  }

  void CharucoBoardTargetOptionsWidget::SetCharucoBoardTargetOptions(const CharucoBoardFeatureExtractor::Options& options) {
    columns_edit_->setValue(options.columns);
    rows_edit_->setValue(options.rows);
    marker_size_edit_->setValue(options.marker_size);
    square_size_edit_->setValue(options.square_size);
    marker_num_edit_->setValue(options.marker_num);
    board_index_edit_->setValue(options.board_index);
    for (size_t i = 0; i < calibration_targets::ArucoTypes().size(); i++) {
      const auto& name_type = calibration_targets::ArucoTypes()[i];
      if (name_type.second == options.aruco_type) {
        aruco_type_edit_->setCurrentIndex(i);
        break;
      }
    }
  }

  CharucoBoardFeatureExtractor::Options CharucoBoardTargetOptionsWidget::CharucoBoardTargetOptions() {
    CharucoBoardFeatureExtractor::Options options;
    options.columns = columns_edit_->value();
    options.rows = rows_edit_->value();
    options.marker_size = marker_size_edit_->value();
    options.square_size = square_size_edit_->value();
    options.marker_num = marker_num_edit_->value();
    options.board_index = board_index_edit_->value();
    options.aruco_type = calibration_targets::ArucoTypeFromName(aruco_type_edit_->currentText().toStdString());
    return options;
  }

}