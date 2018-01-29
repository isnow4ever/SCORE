#include <QApplication>
#include "dialog.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QApplication::setStyle("cleanlooks");
    Dialog w;
    w.show();
    
    return a.exec();
}
