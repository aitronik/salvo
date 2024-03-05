#pragma once

#include <QMainWindow>
#include <QListView>
#include <QHBoxLayout>
#include <QString>
#include <QStandardItemModel>
#include <QMetaObject>
#include <QMetaMethod>
#include <QObject>
#include <QCheckBox>
#include <QLabel>
#include <QDebug>
#include <QSortFilterProxyModel>

#include "core/abstract_gui.h"

/**
 * @brief A class representing the graphical user interface using Qt.
 * 
 * This class provides a graphical user interface for displaying log messages and node information using Qt.
 */
class QtGui : public QObject, public AbstractGui {
    Q_OBJECT

private:
    QMainWindow mainWindow; /**< Main window of the GUI. */
    QWidget centralWidget; /**< Central widget of the GUI. */
    QVBoxLayout centralLayout; /**< Central layout of the GUI. */
    QLabel title1; /**< Title label 1. */
    QLabel title2; /**< Title label 2. */
    QLabel fileLabel; /**< Label for the selected file. */
    QHBoxLayout optionsLayout; /**< Layout for options. */
    QListView listView1; /**< List view 1 for log messages. */
    QListView listView2; /**< List view 2 for node information. */
    QStandardItemModel listView1Model; /**< Model for list view 1. */
    QStandardItemModel listView2Model; /**< Model for list view 2. */
    QSortFilterProxyModel* listView2proxyModel; /**< Proxy model for list view 2. */
    QHBoxLayout horizontalTitle1Layout; /**< Horizontal layout for title 1. */
    QHBoxLayout horizontalTitle2Layout; /**< Horizontal layout for title 2. */
    QVBoxLayout listView1Layout; /**< Vertical layout for list view 1. */    
    QVBoxLayout listView2Layout; /**< Vertical layout for list view 2. */    
    QHBoxLayout listViewsLayout; /**< Horizontal layout for list views. */
    QHBoxLayout footerLayout; /**< Footer layout. */
    QItemSelectionModel* selectionModel; /**< Selection model for list views. */
    QHBoxLayout layout; /**< Main layout. */

public:
    /**
     * @brief Constructor for QtGui.
     */
    QtGui();

    /**
     * @brief Destructor for QtGui.
     */
    ~QtGui();

    /**
     * @brief Resets the list of log messages.
     * 
     * @param listOfMessages The updated list of log messages.
     */
    virtual void resetList(std::list<std::string>& listOfMessages) override;

    /**
     * @brief Updates the log with a new message.
     * 
     * @param message The log message to update.
     */
    virtual void updateLog(std::string message) override;

    /**
     * @brief Updates the node information.
     * 
     * @param message The node message to update.
     * @param r The red component of the color.
     * @param g The green component of the color.
     * @param b The blue component of the color.
     * @param newNode Flag indicating whether it's a new node.
     * @param index The index of the node.
     */
    virtual void updateNodes(std::string message, int r, int g, int b, bool newNode, int index) override;

private slots:

    /**
     * @brief Slot to handle item clicks in listView1.
     * 
     * @param index The index of the clicked item.
     */
    void handleListView1ItemClicked(const QModelIndex& index);
    
public slots:

    /**
     * @brief Resets listView2 with the given list of items.
     * 
     * @param list The list of items to reset.
     */
    void resetListView2(const QStringList& list);

    /**
     * @brief Clears both list views.
     */
    void clearListViews();
    
    /**
     * @brief Slot to add items to listView1.
     * 
     * @param itemText The text of the item to add.
     * @param qColor The color of the item.
     * @param newNode Flag indicating whether it's a new node.
     * @param index The index of the item.
     */
    void addItemToListView1(const QString& itemText, QColor qColor, bool newNode, int index);

    /**
     * @brief Slot to add items to listView2.
     * 
     * @param itemText The text of the item to add.
     */
    void addItemToListView2(const QString& itemText );

    /**
     * @brief Slot to handle checkbox clicks.
     * 
     * @param checked Flag indicating whether the checkbox is checked.
     * @param checkBoxNumber The number of the checkbox.
     */
    void handleCheckBoxClick(bool checked, int checkBoxNumber);

    /**
     * @brief Slot to handle the select-all checkbox click.
     * 
     * @param checked Flag indicating whether the select-all checkbox is checked.
     */
    void handleCheckBoxSelectAllClick(bool checked);
};
