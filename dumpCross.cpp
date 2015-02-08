#include  <iostream>
#include  <shapefil.h>
#include    "network.h"

using namespace std;
int main(int argc, char *argv[])
{
    if  ( argc < 2 )
    {
        cout << "need road shp"<<endl;
        return 1;
    }
    Network network;
    if (!network.load(argv[1]))
    {
        cerr << "can not open " << argv[1];
    }

    SHPHandle shp = SHPCreate("cross", SHPT_POINT);
    DBFHandle dbf = DBFCreate("cross");
    DBFAddField(dbf, "ID", FTString, 11, 0);
    for(int i = 0; i < network.cross_bound(); ++i)
    {
        Cross const &c = network.cross(i);
        double x[] = {c.x};
        double y[] = {c.y};
        SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, x, y, nullptr);
        const int insert = -1;
        int newID = SHPWriteObject(shp, insert, obj);
        DBFWriteStringAttribute(dbf, newID, 0, c.dbId.c_str());
        SHPDestroyObject(obj);
    }
    SHPClose(shp);
    DBFClose(dbf);
    return 0;
}
