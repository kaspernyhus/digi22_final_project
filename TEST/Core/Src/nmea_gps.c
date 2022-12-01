/*
 * gps.c
 *
 *  Created on: 29. nov. 2022
 *      Author: lpjensen
 *
 *      Parts of the code is taken from u-blox.com opensource GPS roll-over fix
 *      "GPS week number roll-over workaround - Application Note"
 */

#include "nmea_gps.h"

extern UART_HandleTypeDef huart2;
#define TIMEZONEDIFF 1

void formatData(float time, gps_data_t* gps_data)
{
	gps_data->time.hours = (int)time/10000;
	gps_data->time.min = (int)(time-(gps_data->time.hours*10000))/100;
	gps_data->time.sec = (int)(time-((gps_data->time.hours*10000)+(gps_data->time.min*100)));

	gps_data->time.hours += TIMEZONEDIFF;
	if(gps_data->time.hours > 23)
	{
		gps_data->time.hours = 0;
		gps_data->date += 10000;
	}
}

void getLocation(gps_data_t* gps_data, uint8_t* rxData)
{
	char txData[750];
	char gpsPayload[100];
	int msgIndex = 0;
	strcpy(txData, (char*)(rxData));
	char *ptr = strstr(txData, "GPRMC");
	if(*ptr == 'G')
	{
		while(1)
		{
			gpsPayload[msgIndex] = *ptr;
			msgIndex++;
			*ptr = *(ptr+msgIndex);
			if(*ptr == '\n')
			{
				gpsPayload[msgIndex] = '\0';
				break;
			}
		}
		float gps_time;
		sscanf(gpsPayload, "GPRMC,%f,A,%f,N,%f,E,%f,%f,%u,", &gps_time, &gps_data->lati, &gps_data->longi, &gps_data->speed, &gps_data->course, &gps_data->date);
		formatData(gps_time, gps_data);
		HAL_Delay(1);
	}
}

 // known day_of_year for each month:
 // Major index 0 is for non-leap years, and 1 is for leap years
 // Minor index is for month number 1 .. 12, 0 at index 0 is number of days before January
static const uint16_t month_days[2][13] = {
		{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
		{ 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }
	};

 // Count the days since start of 1980
 // Counts year * 356 days + leap days + month lengths + days in month
 // The leap days counting needs the "+ 1" because GPS year 0 (i.e. 1980) was a leap year
 uint16_t day_number_1980(uint16_t year, uint16_t month, uint16_t day)
 {
	 uint16_t gps_years = year - 1980;
	 uint16_t leap_year = (gps_years % 4 == 0) ? 1 : 0;
	 uint16_t day_of_year = month_days[leap_year][month - 1] + day;
	 if (gps_years == 0)
		 return day_of_year;
	 return gps_years * 365 + ((gps_years - 1) / 4) + 1 + day_of_year;
 }

 // Convert day_number since start of 1980 to year, month, and day:
 // - integer division of (day_number - 1) by 365.25 gives year number for 1980 to 2099
 // - day number - (year number * 365 days + leap days) gives day of year
 //   The leap days needs "+ 1" because GPS year 0 (i.e. 1980) was a leap year
 // - (day_of_year - 1) / 31 + 1 gives lower limit for month, but this  may be one too low
 //    the guessed month is adjusted by checking the month lengths
 // - days in month is left when the month lengths are subtracted
 // - year must still be adjusted by 1980
 void date_1980(uint16_t day_number, uint16_t *year, uint16_t *month, uint16_t *day)
 {
	 uint16_t gps_years = ((day_number - 1) * 100) / 36525;
	 uint16_t leap_year = (gps_years % 4 == 0) ? 1 : 0;
	 uint16_t day_of_year = day_number;
	 if (gps_years > 0)
	 day_of_year = day_number - (gps_years * 365 + ((gps_years - 1) / 4) + 1);
	 uint16_t month_of_year = (day_of_year - 1) / 31 + 1;
	 if (day_of_year > month_days[leap_year][month_of_year])
	 month_of_year++;
	 *day = day_of_year - month_days[leap_year][month_of_year - 1];
	 *month = month_of_year;
	 *year = 1980 + gps_years;
 }

 // Convert NMEA $GPRMC date string to integer components
 void gprmc2int(char gprmc[], uint16_t *year, uint16_t *month, uint16_t *day)
 {
	  *day = 10 * (gprmc[0] - '0') + (gprmc[1] - '0');
	  *month = 10 * (gprmc[2] - '0') + (gprmc[3] - '0');
	  *year = 10 * (gprmc[4] - '0') + (gprmc[5] - '0');
	  assert(*year >= 0 && *year <= 99 && *month >= 1 && *month <= 12 && *day >= 1 && *day <= 31);
	  // NMEA $GPRMC year number has only 2 digits
	  if (*year > 79)
	  *year = *year + 1900;
	  else
	  *year = *year + 2000;
 }

 // Convert integer date components to NMEA $GPRMC date string
 char *int2gprmc(uint16_t year, uint16_t month, uint16_t day)
 {
	 assert(year >= 1980 && year <= 2079 && month >= 1 && month <= 12
	  && day >= 1 && day <= 31);
	  year = year % 100; // use only decades and years, drop centuries
	  static char gprmc[7];
	  gprmc[0] = '0' + (day / 10);
	  gprmc[1] = '0' + (day % 10);
	  gprmc[2] = '0' + (month / 10);
	  gprmc[3] = '0' + (month % 10);
	  gprmc[4] = '0' + (year / 10);
	  gprmc[5] = '0' + (year % 10);
	  gprmc[6] = '\0';

    return gprmc;
}

 unsigned int rolloverDateConvertion(unsigned int date)
 {
	 char buf[8];
	 uint16_t year, month, day;
	 // Note that the year number in the NMEA string has only two digits
	 utoa(date, buf, 10);
	 char *gprmc = buf;

	 // first, convert NMEA date to integer format (adds century to the year)
	 gprmc2int(gprmc, &year, &month, &day);

	 // calculate how many days there are since start of 1980
	 uint16_t day_num = day_number_1980(year, month, day);

	 // adjust date only if it is before previous GPS week number roll-over
	 // which happened 2019-04-06. That is 14341 days after start of 1980
	 if (day_num <= 14341)
		 day_num = day_num + 1024 * 7;

	 // convert the day number back to integer year, month and day
	 date_1980(day_num, &year, &month, &day);

	 // convert the year, month and day to NMEA string format (drops century from the year)
	 char *adjusted = int2gprmc(year, month, day);

	 int rtn = atoi(adjusted);
	 return (unsigned int)rtn;
 }




