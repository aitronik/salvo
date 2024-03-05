#include <QStringList>
#include <QFileDialog> // Add this line for QFileDialog
#include <QLineEdit>
#include <QComboBox>

#include "impl/qt_gui.h"


QtGui::QtGui() : centralLayout(&centralWidget){

    mainWindow.setWindowTitle("Guardo");


    
    fileLabel.setText("Selected File: None");
    // Create the combo box
    QComboBox* comboBox = new QComboBox();

    // Add the items
    comboBox->addItem("Load File");
    comboBox->addItem("Playback File");
    comboBox->addItem("ROS1");
    comboBox->addItem("ROS2");
    comboBox->addItem("ZMQ");

    // connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int index){
    connect(comboBox, QOverload<int>::of(&QComboBox::activated), [=](int index){
        if (index == 0) {
            // Open a file dialog to select a file
            QString filePath = QFileDialog::getOpenFileName(&mainWindow, "Select File", "", "All Files (*)");
            if (!filePath.isEmpty()) {
                // Update the text label with the selected file name
                fileLabel.setText("Selected File: " + filePath);
                std::cout << "publish to manager CSV" <<std::endl;
                publishSubscriptionChoice(0,filePath.toStdString());
                QMetaObject::invokeMethod(this, "clearListViews");
            }
        }
        if (index == 1){
            QString filePath = QFileDialog::getOpenFileName(&mainWindow, "Select File", "", "All Files (*)");
            if (!filePath.isEmpty()) {
                // Update the text label with the selected file name
                fileLabel.setText("Selected File: " + filePath);
                std::cout << "publish to manager CSV" <<std::endl;
                publishSubscriptionChoice(1,filePath.toStdString());
                QMetaObject::invokeMethod(this, "clearListViews");

            }
        }
        if (index == 3){
            std::cout << "publish to manager ROS2" <<std::endl;
            publishSubscriptionChoice(3,"");
            QMetaObject::invokeMethod(this, "clearListViews");
        }
        

    });

 
    // Add the options list view and the file label as the first row of listViewsLayout
    optionsLayout.addWidget(comboBox);
    optionsLayout.addWidget(&fileLabel);

    optionsLayout.setStretchFactor(comboBox, 1); // Takes 1/3 of the space
    optionsLayout.setStretchFactor(&fileLabel, 3); // Takes 1/3 of the space


    // Create QLabel widgets for the titles
    

    
    // listView1Layout = [ HorizontalTitle1Layout[ TITLE - Select ALL ]
    //                        listView1         ]

    // listView2Layout = [     Title 2 
    //                          listView2         ]

    // listViewsLayout = [listView1Layout | listView2Layout ]    
 
    title1.setText("Nodes");
    
    QCheckBox* checkBoxSelectAll = new QCheckBox("Select All");
    checkBoxSelectAll->setChecked(true);
    QObject::connect(checkBoxSelectAll, &QCheckBox::clicked, [this](bool checked) {
            handleCheckBoxSelectAllClick(checked);
        });
    // Assuming you have a QHBoxLayout* named layout
    QSpacerItem* verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Minimum);

    horizontalTitle1Layout.addSpacerItem(verticalSpacer);
    horizontalTitle1Layout.addWidget(&title1);
    horizontalTitle1Layout.addWidget(checkBoxSelectAll);



    title2.setText("Messages");
    QLabel* searchLabel = new QLabel("Search");
    searchLabel->setAlignment(Qt::AlignRight);
    // Create the search box
    QLineEdit* searchBox = new QLineEdit();
    // Connect the textChanged signal of the search box to a slot that updates the filter
    connect(searchBox, &QLineEdit::textChanged, [=](QString text){
        listView2proxyModel->setFilterRegExp(QRegExp(text, Qt::CaseInsensitive, QRegExp::FixedString));
    });

 
    horizontalTitle2Layout.addSpacerItem(verticalSpacer);

    horizontalTitle2Layout.addWidget(&title2);
    horizontalTitle2Layout.addWidget(searchLabel);
    horizontalTitle2Layout.addWidget(searchBox);

    horizontalTitle2Layout.setStretchFactor(&title2, 1); // Takes 1/3 of the space
    horizontalTitle2Layout.setStretchFactor(searchLabel, 2); // Takes 1/3 of the space
    horizontalTitle2Layout.setStretchFactor(searchBox, 2); // Takes 2/3 of the

  
    // Initialize the QStandardItemModel
    listView1.setModel(&listView1Model);

    
    listView2proxyModel = new QSortFilterProxyModel(this);
    listView2proxyModel->setSourceModel(&listView2Model);
    listView2.setModel(listView2proxyModel);


    // Set minimum sizes for the list views
    listView1.setMinimumSize(200, 200);
    listView2.setMinimumSize(200, 200);

    listView1.setSelectionMode(QAbstractItemView::MultiSelection);

    // Create a QVBoxLayout for each list view with a title above
    listView1Layout.addLayout(&horizontalTitle1Layout);
    listView1Layout.addWidget(&listView1);
     

    listView2Layout.addLayout(&horizontalTitle2Layout);
    listView2Layout.addWidget(&listView2);

    
    listViewsLayout.addLayout(&listView1Layout);
    listViewsLayout.addLayout(&listView2Layout);
    listViewsLayout.setStretchFactor(&listView1Layout, 1); // Takes 1/3 of the space
    listViewsLayout.setStretchFactor(&listView2Layout, 2); // Takes 2/3 of the


    // Create and add 7 QCheckBox widgets with text labels to the footer layout
    for (int i = 0; i < 5; ++i) {
        QCheckBox* checkBox = new QCheckBox(getEventStringbyId(i).c_str());
        checkBox->setChecked(true); // Set checkboxes as activated (checked) by default
        
        QObject::connect(checkBox, &QCheckBox::clicked, [this, i](bool checked) {
            handleCheckBoxClick(checked, i);
        });
        
    
        footerLayout.addWidget(checkBox);
    }
 
    centralLayout.addLayout(&optionsLayout);
    // Add a vertical spacer to create some space between the options and the rest of the layout
    centralLayout.addSpacing(10);
    centralLayout.addLayout(&listViewsLayout);
    
    // Add some spacing between list views and footer
    centralLayout.addSpacing(10);

    // Add the footer layout to the central layout
    centralLayout.addLayout(&footerLayout);

    // Set the central layout for the central widget
    centralWidget.setLayout(&centralLayout);
    

    // Get the selection model of listView1
    selectionModel = listView1.selectionModel();


    // Connect the clicked signal to handleListView1ItemClicked slot
    QObject::connect(&listView1, &QListView::clicked, this, &QtGui::handleListView1ItemClicked);

    mainWindow.setCentralWidget(&centralWidget);
    mainWindow.show();

}

QtGui::~QtGui() {
    // No need to delete non-pointer objects; Qt will handle their destruction automatically
}

void QtGui::resetList(std::list<std::string>& listOfMessages){
    QStringList list;
    for (auto msg : listOfMessages){
        list << msg.c_str();
    }
    QMetaObject::invokeMethod(this, "resetListView2", Qt::QueuedConnection, Q_ARG(QStringList, list));
}

void QtGui::updateLog(std::string message) {
    QString qStr = QString::fromStdString(message);
    QMetaObject::invokeMethod(this, "addItemToListView2", Qt::QueuedConnection, Q_ARG(QString, qStr));
}

void QtGui::updateNodes(std::string message,  int r,int g, int b, bool newNode,int index) {

    QColor qColor(r, g, b);
    QString qStr = QString::fromStdString(message);
    QMetaObject::invokeMethod(this, "addItemToListView1", Qt::QueuedConnection,
                                                            Q_ARG(QString, qStr),  
                                                            Q_ARG(QColor, qColor),
                                                            Q_ARG(bool, newNode),
                                                            Q_ARG(int, index));

}

// Slot to add items to listView1
void QtGui::addItemToListView1(const QString& itemText,  QColor qColor, bool newNode,int index) {
    
    QBrush brush(qColor); // Set text color to red

    if (newNode){
        QStandardItem* item = new QStandardItem(itemText);
        item->setForeground(brush);
        listView1Model.appendRow(item);
    }
    else{
        QModelIndex index_i = listView1Model.index(index, 0); // Row 1, Column 0
        QStandardItem* item_i = listView1Model.itemFromIndex(index_i);
    
        if (item_i) {
            item_i->setData(itemText, Qt::DisplayRole);
            item_i->setForeground(brush);
        }
    }
    listView1.viewport()->update();
    listView1.scrollToBottom();
}

// Slot to add items to listView2
void QtGui::addItemToListView2(const QString& itemText ) {
    QStandardItem* item = new QStandardItem(itemText);
    listView2Model.appendRow(item);
    // you cannot scrollToBottom too fast, otherwise you get a slowdown
    // TODO: implement a timer that scrolls
    // listView2.scrollToBottom(); 
}

void QtGui::clearListViews(){    
     // Clear the listView Models
    listView1Model.clear();
    listView2Model.clear();
}


void QtGui::resetListView2(const QStringList& list){
    
     // Clear the listView2Model
    listView2Model.clear();
    
    // Add all elements from the QStringList to listView2Model
    foreach (const QString& item, list) {
        QStandardItem* listItem = new QStandardItem(item);
        listView2Model.appendRow(listItem);
    }

    // Scroll to the bottom of the list view
    listView2.scrollToBottom();

}

void QtGui::handleCheckBoxClick(bool checked, int checkBoxNumber) {

    // if (checked) {
    //     std::cout << "Checkbox" << checkBoxNumber << "clicked and activated" << std::endl;
    //     // Place your code to execute when the checkbox is clicked and activated here
    // } else {
    //     std::cout << "Checkbox" << checkBoxNumber << "clicked and deactivated" << std::endl;
    //     // Place your code to execute when the checkbox is clicked and deactivated here
    // }
    
    publishFilterEvent(checkBoxNumber,checked);
}
void QtGui::handleCheckBoxSelectAllClick(bool checked){
    publishSelectAllNodes(checked);
}

void QtGui::handleListView1ItemClicked(const QModelIndex& index) {

   
    if (index.isValid()) {
        int row = index.row();
        // Check if the clicked item is selected
        bool active = selectionModel->isSelected(index); 
        publishFilterNode(row, active );
    }
}