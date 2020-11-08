import sys
import math


search_data = '$GPGGA'
'''

위도 : Latitude (Y)
경도 : Longitude (X)

Created Date: Oct 26. 2020
Copyright: UNMMAND SOLUTION
Author: Dae Jong Jin 
Description: GPS NMEA DATA to TM (Transverse Mercator) from TXT file

'''

# # 장반경
# m_arMajor = 6377397.155
# # 단반경
# m_arMinor =  6356078.9633422494


# 장반경
m_arMajor = 6378137
# 단반경
m_arMinor = 6356752.31425

# 원점축척계수
m_arScaleFactor = 1.0 

# Y축 원점 가산값 
m_arFalseNorthing = 600000.0
# X축 원점 가산값 
m_arFalseEasting = 200000.0

# 제 1이심률
m_Es = 1 - math.pow((m_arMinor/m_arMajor),2)
# 제 2이심률
m_Esp = math.pow((m_arMajor/m_arMinor),2) - 1


class GeoPoint :
	def __init__(self, x=0, y=0, z = 0) :
		self.x = x
		self.y = y
		self.z = z

	@property
	def X(self) :
		return self.x
	@property
	def Y(self) :
		return self.y
	@property
	def Z(self) :
		return self.z

	@X.setter
	def X(self, x) :
		self.x = x
	@Y.setter
	def Y(self, y) :
		self.y = y
	@Z.setter
	def Z(self, z) :
		self.z = z

def gps2dms(data) :
    gps_data = float(data)
    deg_data = int(gps_data/100)
    min_data = gps_data - deg_data * 100
    sec_data = (gps_data - int(gps_data)) * 60

    ret = deg_data + min_data/60

    return ret, deg_data, int(min_data), sec_data

def deg2rad( degree ) :
	return degree * math.pi / 180.0

def rad2deg( radian ) :
	return radian * 180.0 / math.pi

def e0fn( x ) :
	return 1.0 - 0.25 * x * (1.0 + x / 16.0 * (3.0 + 0.75 * x))

def e1fn( x ) :
	return 0.375 * x * (1.0 + 0.25 * x * (1.0 + 0.46875 * x))

def e2fn( x ) :
	return 0.05859375 * x * x * (1.0 + 0.75 * x)

def e3fn( x ) :
	return x * x * x * (35.0 / 3072.0)

def muxfn( e0,  e1,  e2,  e3,  phi ):
    return e0 * phi - e1 * math.sin(2.0 * phi) + e2 * math.sin(4.0 * phi) - e3 * math.sin(6.0 * phi)


# 투영원점 경도(rad) 127 * pi / 180
m_arLonCenter = deg2rad(127)
# 투영원점 위도(rad) 38 * pi / 180
m_arLatCenter = deg2rad(38)

m_M0 = m_arMajor * muxfn(e0fn(m_Es), e1fn(m_Es), e2fn(m_Es), e3fn(m_Es), m_arLatCenter) 

def geo2tm( in_pt, out_pt ) :
    # input_pt = GeoPoint()

    delta_lon = in_pt.X - m_arLonCenter
    sin_phi = math.sin(in_pt.Y)
    cos_phi = math.cos(in_pt.Y)
    tan_phi = math.tan(in_pt.Y)

    m_T = math.pow(tan_phi,2)
    m_C = (m_Es / (1-m_Es)) * cos_phi * cos_phi 
    m_A = delta_lon * cos_phi
    m_As = m_A * m_A
    m_N = m_arMajor / math.sqrt(1- (m_Es * sin_phi * sin_phi))

# 자오선 길이
    m_M = m_arMajor * muxfn(e0fn(m_Es), e1fn(m_Es), e2fn(m_Es), e3fn(m_Es), in_pt.Y) 

    out_pt.X = m_arFalseNorthing + (m_arScaleFactor * (m_M - m_M0 + m_N * tan_phi * (m_As * (0.5 + m_As / 24.0 * (5.0 - m_T + 9.0 * m_C + 4.0 * math.pow(m_C,2) + m_As / 30.0 * (61 - 58.0 * m_T + math.pow(m_T, 2) + 600.0 * m_C - 330.0 * m_Esp))))))
    out_pt.Y = m_arFalseEasting + (m_arScaleFactor * m_N * m_A * (1.0 + m_As / 6.0 * (1.0 - m_T + m_C + m_As / 20.0 * (5.0 - 18.0 * m_T + m_T * m_T + 72.0 * m_C - 58.0 * m_Esp))))

    return out_pt

raw_pt = GeoPoint()
output = GeoPoint()
testput = GeoPoint()


# lat, lat_deg, lat_min, lat_sec = gps2dms(3730.0308)
# long, long_deg, long_min, long_sec = gps2dms(12655.2369)


# raw_pt.X = deg2rad(long)
# raw_pt.Y = deg2rad(lat)

# output = geo2tm(raw_pt, output)
# # print(raw_pt.X, raw_pt.Y)
# # print(output.X, output.Y)

# # test = ','.join(data).replace("b","")

# print("lat : {} long : {} ".format(lat , long), end="")
# print("{}도 {}분 {}초".format(lat_deg,lat_min,lat_sec))
# print("{}도 {}분 {}초".format(long_deg,long_min,long_sec))
# print("X : {}[m] Y : {}[m]".format(output.X , output.Y))



try :
    with open('sample/nmea_sample_1.txt', 'r') as f:
        read_txt = f.readlines()
    data_list = []

    for i in read_txt:
        my_list = i.split(",")
        data_list.append(my_list)

    cnt = 0
    with open("sample/output.txt", "w") as file:
        for data_index in data_list:
            if search_data in data_index:
                datas = data_index
                cnt += 1
                if len(datas[0]) < 9:
                    try:
                        for line in datas :
                            if search_data in line:
                                data = datas
  
                                Lat_data = data[2]
                                Long_data = data[4]
                                # print(data)

                                if data[3] == "S":
                                    Lat_data = -float(data[2])
                                elif data[5] == "W":
                                    Long_data = -float(data[4])

                                lat, lat_deg, lat_min, lat_sec = gps2dms(Lat_data)
                                long, long_deg, long_min, long_sec = gps2dms(Long_data)

                                raw_pt.X = deg2rad(long)
                                raw_pt.Y = deg2rad(lat)
                                output = geo2tm(raw_pt, output)
                                print(cnt, output.X, output.Y)

                                file.write("{} {} {} \n".format(cnt, output.X, output.Y))
                                file.flush()
                            # print(raw_pt.X, raw_pt.Y)


                            # test = ','.join(data).replace("b","")

                            # print("lat : {} long : {}".format(lat , long))
                            # print("{}도 {}분 {}초".format(lat_deg,lat_min,lat_sec))
                            # print("{}도 {}분 {}초".format(long_deg,long_min,long_sec))
                    except Exception as e:
                        cnt = cnt - 1
                        pass

except Exception as e:
    print(e)
