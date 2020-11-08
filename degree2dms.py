import sys
latitude = 3730.0308
longitude = 12655.2369

'''
Created Date: Oct 06. 2020
Copyright: UNMMAND SOLUTION
Author: Dae Jong Jin 
Description: GPS Degree to DMS(도분초) Convertion 
'''


def gps2dms(data) :
    gps_data = float(data)
    deg_data = int(gps_data/100)
    min_data = gps_data - deg_data * 100
    sec_data = (gps_data - int(gps_data)) * 60

    ret = deg_data + min_data/60

    return ret, deg_data, int(min_data), sec_data

def main():

    while True :
        try :
            in_lat, in_long = map(float, input("위도, 경도를 입력하세요 : ").split())

            print(in_lat, in_long)

            lat, lat_deg, lat_min, lat_sec = gps2dms(in_lat)
            long, long_deg, long_min, long_sec = gps2dms(in_long)
            print("lat : {} long : {} ".format(lat, long))
            print("{}도 {}분 {}초".format(lat_deg,lat_min,lat_sec))
            print("{}도 {}분 {}초".format(long_deg,long_min,long_sec))
        except KeyboardInterrupt :
            print("Press ctrl + c")
            sys.exit()
        except :
            print("잘못 입력하셨습니다.")
        
if __name__ == "__main__":
    main()