import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtCore import QMutex

from .data_receive import create_node

def qtcreate_node(qt, node_attr):
    t_ = threading.Thread(target=create_node, 
                        args=(qt.nodes, qt.data, qt.mutex, node_attr))
    t_.daemon = True
    t_.start()

class RealTimePlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        # 设置窗口标题
        self.setWindowTitle("实时绘图程序")
        # 创建一个绘图窗口
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        
        self.mutex = QMutex()   
        self.data: dict = {}
        self.nodes: dict = {}
        self.buttons: dict = {}
        self.create_button("auto range", self.click_auto_scroll)
        
        self.data['motor_fdb'] = [0] * 1000
        self.data['motor_ref'] = [0] * 1000
        
        qtcreate_node(self,"data_process")

        # 创建数据列表
        self.x = list(range(1000))

        # 创建一个曲线对象
        self.curve = self.graphWidget.plot(self.x, self.data['motor_fdb'])

        # 创建一个定时器，每10毫秒更新一次数据
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)

    def update_plot(self):
        self.x = self.x[1:]
        self.x.append(self.x[-1] + 1)
        self.mutex.lock()
        self.curve.setData(self.x, self.data['motor_fdb'])
        self.mutex.unlock()

    def create_button(self, text, callback):
        self.buttons["text"] = QPushButton(text, self)
        self.buttons["text"].clicked.connect(callback)
    
    def click_auto_scroll(self):
        self.graphWidget.autoRange()

    def closeEvent(self, event):
        """窗口关闭事件"""
        print("Closing application...")
        # 关闭 ROS 2 节点
        for node in self.nodes.values():
            node.destroy_node()
        super().closeEvent(event)

    

def main(args=None):
    app = QApplication(sys.argv)
    plotter = RealTimePlotter()
    plotter.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()