#ifndef NAVI_H_
#define NAVI_H_

#define COMPASS_X	180
#define COMPASS_Y	60
#define COMPASS_R	35
#define HORIZON_X	COMPASS_X
#define HORIZON_Y	COMPASS_Y

#define MIN_TO_RADF	M_PI/180/60/10000
// 6357752 promien biegunowy
// 6378127 promien rownikowy
#define EARTH_R		6371008 // mean radius
#define WPT_NAME_LEN	8

typedef struct
{
	int32_t 	lat;
	int32_t 	lon;
	char		name[WPT_NAME_LEN];
} wpt_t;

wpt_t dest_wpt;
void openNavi(void);
void closeNavi(void);
void btnSaveTrk(void);
void btnSaveWpt(void);
void btnNav2Wpt(void);
uint32_t distance_to(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
uint32_t course_to(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
void gps_convert(void);

#endif /* NAVI_H_ */