인터페이스 만들 때 주의사항

패키지는 ament_cmake 필수/ ament_python 안됨
ament_python 기준 package.xml에 <exec_depend>인터페이스 패키지 이름</exec_depend> 추가 "권장"/ 없어도 작동
ament_cmake 인터페이스 패키지의 CMakeLists.txt에 다음 추가
```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```
같은 패키지에 package.xml에 다음 추가
```
  <depend>geometry_msgs</depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

-> 빌드할 때 인터페이스 패키지를 인식하여 다른 패키지에서도 인식 가능