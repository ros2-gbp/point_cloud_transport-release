%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-point-cloud-transport-py
Version:        2.0.6
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS point_cloud_transport_py package

License:        BSD
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-iron-pluginlib
Requires:       ros-iron-point-cloud-transport
Requires:       ros-iron-pybind11-vendor
Requires:       ros-iron-rclcpp
Requires:       ros-iron-rpyutils
Requires:       ros-iron-sensor-msgs
Requires:       ros-iron-ros-workspace
BuildRequires:  ros-iron-ament-cmake-python
BuildRequires:  ros-iron-ament-cmake-ros
BuildRequires:  ros-iron-pluginlib
BuildRequires:  ros-iron-point-cloud-transport
BuildRequires:  ros-iron-pybind11-vendor
BuildRequires:  ros-iron-python-cmake-module
BuildRequires:  ros-iron-rclcpp
BuildRequires:  ros-iron-sensor-msgs
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Python API for point_cloud_transport

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DAMENT_PREFIX_PATH="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Thu May 16 2024 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.6-1
- Autogenerated by Bloom

* Tue Mar 26 2024 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.5-1
- Autogenerated by Bloom

* Wed Feb 14 2024 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.4-1
- Autogenerated by Bloom

* Wed Feb 14 2024 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.3-2
- Autogenerated by Bloom

* Tue Dec 12 2023 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.3-1
- Autogenerated by Bloom

* Thu Oct 05 2023 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.2-1
- Autogenerated by Bloom

* Wed Sep 20 2023 Alejandro Hernández <alejandro@openrobotics.org> - 2.0.1-1
- Autogenerated by Bloom

