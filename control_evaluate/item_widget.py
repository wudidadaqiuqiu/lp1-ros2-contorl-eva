import inspect
from PyQt5.QtWidgets import (QHBoxLayout, QLabel, QWidget, QFrame, 
    QPushButton, QListWidgetItem, QApplication, QLineEdit)
# from .signal_eval import *

class ItemWidget(QWidget):
    def __init__(self, type, list_item: QListWidgetItem, parent=None):
        super().__init__(parent)
        self.list_item = list_item  # 保存关联的 QListWidgetItem
        self.sub_dict = {}
        self.processor = type()
        self.parent = parent
        # 水平布局
        self.layout = QHBoxLayout(self)

        # 添加文本标签
        self.label = QLabel(type.get_name(), self)
        self.layout.addWidget(self.label)

        self.load_type(type)
        # # 其他控件可以根据需要添加
        self.button = QPushButton("开始", self)
        self.button.clicked.connect(self.start_subscribe)
        self.layout.addWidget(self.button)
        
        # 设置布局
        self.setLayout(self.layout)
        self.setAutoFillBackground(True)
        # 启用选择
        self.setStyleSheet("background-color: lightgray;")
        # self.setAcceptDrops(True)

    def load_type(self, type: type):
        # 获取 evaluate 函数的签名
        si = inspect.signature(type.evaluate)
        
        # 创建控件并连接到处理函数
        self.line_edits = {}  # 保存 QLineEdit 控件，用于后续访问
        for param in si.parameters:
            line_edit = QLineEdit(self)
            line_edit.setPlaceholderText(param)
            line_edit.textChanged.connect(lambda text, param=param: self.handle_input(param, text))
            self.layout.addWidget(line_edit)
            self.line_edits[param] = line_edit

    def get_subscirbe_dict(self):
        return self.sub_dict
    
    def handle_input(self, param, text):
        # 打印输入内容或者处理
        # print(f"Parameter: {param}, Input: {text}")
        self.sub_dict[param] = text
        # print(self.sub_dict)
        # self.parent.update_item(self)

    def start_subscribe(self):
        self.parent.update_item(self)