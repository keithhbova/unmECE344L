# Form implementation generated from reading ui file 'basicCypher.ui'
#
# Created by: PyQt6 UI code generator 6.4.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def performShitfTransformationOnCapitalLetters(asciiCharToShift, shiftNumber = 0):
    shiftContainer = asciiCharToShift - 65
    shiftContainer = shiftContainer + shiftNumber
    
    if(shiftContainer < 0):
        shiftContainer = 26 + shiftContainer
    
    
    if(shiftContainer > 26):
        shiftContainer = shiftContainer - 26
    
    return shiftContainer + 65
    

def performShitfTransformationOnLowercaseLetters(asciiCharToShift, shiftNumber = 0):
    shiftContainer = asciiCharToShift - 97
    shiftContainer = shiftContainer + shiftNumber
    
    if(shiftContainer < 0):
        shiftContainer = 26 + shiftContainer
    
    
    if(shiftContainer > 26):
        shiftContainer = shiftContainer - 26
    
    return shiftContainer + 97

def shiftAllCharactersInString(shiftMe = "", shiftNumber = 0):
    
    listOfAsciiValues = [ord(c) for c in shiftMe]
    listOfShiftedAsciiValues = []
    #print(str(shiftNumber) + "\t" + str(listOfAsciiValues))
    
    for letter in listOfAsciiValues:
        letterIsACapital = (letter>= 65 and letter<=90)
        letterIsALowercase = (letter>= 97 and letter<=122)
        letterIsASpacebar = (letter == 32)
        if(letterIsACapital):
            listOfShiftedAsciiValues.append(performShitfTransformationOnCapitalLetters(letter, shiftNumber))
        if(letterIsALowercase):
            listOfShiftedAsciiValues.append(performShitfTransformationOnLowercaseLetters(letter, shiftNumber))
        if(letterIsASpacebar):
            listOfShiftedAsciiValues.append(32)     
    
    return "".join(chr(i) for i in listOfShiftedAsciiValues)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(708, 192)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(20, 40, 81, 16))
        self.label.setObjectName("label")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(10, 70, 221, 21))
        self.lineEdit.setObjectName("lineEdit")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(300, 40, 91, 16))
        self.label_2.setObjectName("label_2")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(480, 70, 221, 16))
        self.label_4.setObjectName("label_4")
        self.horizontalSlider = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider.setGeometry(QtCore.QRect(270, 70, 160, 22))
        self.horizontalSlider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 708, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        self.horizontalSlider.setRange(-25, 25)
        self.horizontalSlider.valueChanged.connect(self.doSomethingWithSliderValue)
         
        self.lineEdit.textChanged.connect(self.runMeEveryTimeTextChangesFromLineEdit) 
    
    def runMeEveryTimeTextChangesFromLineEdit(self):
        newEncryptedMessage = shiftAllCharactersInString(self.lineEdit.text(), self.horizontalSlider.value())
        self.label_4.setText(newEncryptedMessage)            
        return
                    
    def doSomethingWithSliderValue(self, value):
        self.runMeEveryTimeTextChangesFromLineEdit()
        self.label_2.setText(str(value))
        return   
      

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Encryption Using Serial Ports Except Without Serial Ports"))
        self.label.setText(_translate("MainWindow", "Input Text:"))
        self.label_2.setText(_translate("MainWindow", "0"))
        self.label_4.setText(_translate("MainWindow", "Output:"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())
