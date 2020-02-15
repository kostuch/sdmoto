#include <Arduino.h>
#include "navi.h"

void openNavi() {}
void closeNavi() {}
void btnSaveTrk(void) {}
void btnSaveWpt(void) {}
void btnNav2Wpt(void) {}

uint32_t distance_to(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)		//Obliczenie odleglosci pomiedzy punktami
{
	float dlon = MIN_TO_RADF * (lon2 - lon1);
	float dlat = MIN_TO_RADF * (lat2 - lat1);
	float a = pow(sin(dlat / 2), 2) + cos(MIN_TO_RADF * lat1) * cos(MIN_TO_RADF * lat2) * pow(sin(dlon / 2), 2);
	float c = 2 * atan2(sqrt(a), sqrt(1 - a));
	uint32_t distance = EARTH_R * c;
	return distance;
}

uint32_t course_to(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
	// returns course in degrees (North=0, West=270) from position 1 to position 2,
	// both specified as signed decimal-degrees latitude and longitude.
	// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
	// Courtesy of Maarten Lamers
	double dlon = radians(lon2 - lon1);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	double a1 = sin(dlon) * cos(lat2);
	double a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);

	if (a2 < 0.0)
		a2 += TWO_PI;

	return degrees(a2);
}
/* Convert internal format (minutes * 10000) to
 * DDD° MM' SS.SSS" DDD° MM' SS.SSS"
 * DDD° MM.MMM' DDD° MM.MMM'
 * DDD.DDDDD° DDD.DDDDD° */
/* void gps_convert()
{
	//supported formats					syntax								examples
	//degrees decimal minutes			DDD° MM.MMM' DDD° MM.MMM'			N 47° 38.938 W 122° 20.887				!!!!!!!! GPS !!!!!!!!
	//decimal degrees					DDD.DDDDD° DDD.DDDDD°				N 47.64896° W -122.34812°
	//degrees minutes seconds			DDD° MM' SS.SSS" DDD° MM' SS.SSS"	N 47° 38' 56.292" W 122° 20' 53.232"
	//    decimal_degrees=degrees + minutes/60 + seconds/3600
	//    degrees = decimal_degrees
	//    minutes = 60 ∗ (decimal_degrees − degrees )
	//    seconds = 3600 ∗ (decimal_degrees − degrees) − 60 ∗ minutes
	//    52.09288 -> 52 05 34
	uint32_t temp32;
	//lat
	location.lat_deg = labs(location.latitude) / 600000;						//52.09293 (52 stopnie)
	location.lat_min = (labs(location.latitude) - (location.lat_deg * 600000)) / 10000;	//(31255758-31200000)/10000=5.5758 (5 minut)
	temp32 = labs(location.latitude) - ((uint32_t) location.lat_deg * 600000) - ((uint32_t) location.lat_min * 10000);
	location.lat_sec = 60 * temp32 / 10000;									//60*(55758-50000)/10000 = 34.548 (34 sekundy)
	location.lat_sec_frac = 6 * (labs(location.latitude) - ((uint16_t) location.lat_deg * 600000) - ((uint16_t) location.lat_min * 10000)) % 1000; // 34548 % 1000=548
	// lon
	location.lon_deg = labs(location.longitude) / 600000;
	location.lon_min = (labs(location.longitude) - (location.lon_deg * 600000)) / 10000;
	temp32 = labs(location.longitude) - ((uint32_t) location.lon_deg * 600000) - ((uint32_t) location.lon_min * 10000);
	location.lon_sec = 60 * temp32 / 10000;
	location.lon_sec_frac = 6 * (labs(location.longitude) - ((uint16_t) location.lon_deg * 600000) - ((uint16_t)location.lon_min * 10000)) % 1000;

	if (location.latitude > 0) location.ns = 'N';
	else location.ns = 'S';

	if (location.longitude > 0) location.we = 'W';
	else location.we = 'E';

	location.lat_dec_deg = labs(location.latitude) / 600000.0;
	location.lon_dec_deg = labs(location.longitude) / 600000.0;
	location.lat_dec_min = (labs(location.latitude) - (location.lat_deg * 600000)) / 10000.0;
	location.lon_dec_min = (labs(location.longitude) - (location.lon_deg * 600000)) / 10000.0;
} */