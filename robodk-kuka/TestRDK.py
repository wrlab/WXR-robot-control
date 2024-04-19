from robodk.robolink import *

# RoboDK에 연결
RDK = Robolink()

# 특정 객체의 이름으로 객체를 찾음 (예: "MyObject")
object_name = "untitled1"
my_object = RDK.Item(object_name)

#bbox = my_object.BoundingBox()
bounding = my_object.setParam("BoundingBox")
#bounding.pop('size')
print(bounding.pop('size'))
print(bounding.pop('min'))
# # bounding box로부터 width, height, depth 계산
# width = bbox[1][0] - bbox[0][0]  # Xmax - Xmin
# height = bbox[1][1] - bbox[0][1]  # Ymax - Ymin
# depth = bbox[1][2] - bbox[0][2]  # Zmax - Zmin
#
# print(f"Width: {width}, Height: {height}, Depth: {depth}")