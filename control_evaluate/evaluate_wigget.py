import inspect
import sys
from PyQt5.QtCore import Qt, QStringListModel
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QComboBox, 
    QListWidget, QListView, QPushButton, QLineEdit, QListWidgetItem)
from PyQt5.QtCore import QThread
from .rmse_eval import *
from .item_widget import ItemWidget
from .subscribe_ndoe import TopicCheckerNode, node_init, main

class RosThread(QThread):
    def __init__(self):
        super().__init__()
        self.node, self.topic_check = node_init()
        self.topic_check: TopicCheckerNode
    def run(self):
        main([self.node, self.topic_check])

class ListViewComboBoxApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Add ComboBox Item to ListView')
        self.setGeometry(100, 100, 400, 300)

        self.layout = QVBoxLayout(self)

        # 创建一个 QListView 来显示选中的项
        self.list_widget = QListWidget(self)
        # self.model = QStringListModel()
        # self.list_view.setModel(self.model)
        self.layout.addWidget(self.list_widget)

        # 创建一个 ComboBox，允许用户选择一个项
        self.combo_box = QComboBox(self)
        self.combo_box.addItems(eval_types.keys())
        self.layout.addWidget(self.combo_box)

        # 创建一个按钮，点击后将选中的项添加到 ListView
        self.add_button = QPushButton("Add to List", self)
        self.add_button.clicked.connect(self.add_item_to_list)
        self.layout.addWidget(self.add_button)
        self.ros_thread = RosThread()
        self.ros_thread.start()

    def add_item_to_list(self):
        """将 ComboBox 中当前选中的项添加到 ListView 中"""
        selected_item = self.combo_box.currentText()

        self.add_item(selected_item)
        # 获取当前列表中的项
        # current_items = self.model.stringList()
        # # 如果项还未在列表中，才添加
        # if selected_item not in current_items:
        #     current_items.append(selected_item)
        #     self.model.setStringList(current_items)
    
    def update_item(self, item: ItemWidget):
        a = item.get_subscirbe_dict()
        for v in a.values():
            res = self.ros_thread.topic_check.is_topic_starts_(v)
            print(res)
            
        # self.ros_thread.topic_check.is_topic_starts_()
        print(a)

    def add_item(self, label: str):
        # print(type(label))
        # item_count = self.list_widget.count()  # 获取当前列表项的数量
        # self.list_widget.addItem(f"Item {item_count + 1}")  # 添加新项
        list_item = QListWidgetItem(self.list_widget)
        item_widget = ItemWidget(eval_types[label], list_item, self)
        item_widget.setFixedHeight(50)
        list_item.setSizeHint(item_widget.sizeHint())  # 设置项的大小
        self.list_widget.setItemWidget(list_item, item_widget)
        return item_widget

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ListViewComboBoxApp()
    window.show()
    sys.exit(app.exec_())
