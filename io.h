#ifndef  IO_H
#define  IO_H

#include  <vector>
#include  <string>
#include  <fstream>
#include    "road.h"
#include    "network.h"
#include    "ivmm.h"


struct PreparedGpsRecord{
    std::string carID;
    std::string datatime;
    std::string lngStr;
    std::string latStr;
    std::string xStr;
    std::string yStr;
    GpsPoint gpsPoint;
};

struct RawTrajRecord{
    std::string crossDBID;
    std::string datetime;
    long timestamp;
};

using RawTraj = std::vector<RawTrajRecord>;

std::istream& operator >> (std::istream&, PreparedGpsRecord& d );
std::ostream& operator << (std::ostream&, PreparedGpsRecord const& d );
std::istream& operator >> (std::istream&, RawTrajRecord& d);
std::ostream& operator << (std::ostream&, RawTrajRecord const& d);

std::vector<GpsPoint> loadGpsPoints(std::string const& filename);
std::vector<GpsPoint> loadGpsPoints(std::vector<PreparedGpsRecord> const& ds);
std::vector<PreparedGpsRecord> loadPreparedGpsRecord(std::string const& filename);
void saveToShpPoints(std::string const& filename, std::vector<GpsPoint> const& points);
void preparedGpsLogToShpPoints(std::string const& gpsLog, std::string const& filename);
void savePathToRawText(Path const& path, std::string const& filename);


RawTraj loadTraj(std::string const& filename);
void saveTraj(std::string const& filename, RawTraj const& traj);
Path trajToPath(Network const& network, RawTraj const& traj);
void saveRawTrajToShp(std::string const& filename, RawTraj const& traj);
void savePathToShp(std::string const& filename, Path const& path);

#endif  /*IO_H*/
