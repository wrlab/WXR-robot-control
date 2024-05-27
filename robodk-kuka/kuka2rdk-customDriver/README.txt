kuka2robodk-customDriver 폴더 내용:
KUKA 로봇 컨트롤러 프록시 서버와 통신하기 위한 클라이언트
KUKA 로봇 컨트롤러 1대와 1:1 대응한다
2개의 KUKA 로봇 컨트롤러와 통신하기 위해서 2개의 클라이언트를 실행해야한다
robodk_config_v2.py의 Config_host 는 프록시 서버의 주소 및 포트
rdk_customDriver_Meltio.py 는 커스텀드라이버를 이용한 kuka2rdk 미러링 코드다.