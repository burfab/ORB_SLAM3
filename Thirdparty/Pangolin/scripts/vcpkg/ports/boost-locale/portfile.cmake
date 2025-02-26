# Automatically generated by scripts/boost/generate-ports.ps1

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO boostorg/locale
    REF boost-1.78.0
    SHA512 17870d13898fe1c8df3958757aba81f385924ee063e9f92f821ced912346b89d106b2256d659239323efba125ecd8507a8f3fbc326285cc243befdab5dcaf72d
    HEAD_REF master
    PATCHES
        0001-Fix-boost-ICU-support.patch
        allow-force-finding-iconv.patch
)

if(NOT DEFINED CURRENT_HOST_INSTALLED_DIR)
    message(FATAL_ERROR "boost-locale requires a newer version of vcpkg in order to build.")
endif()
include(${CURRENT_HOST_INSTALLED_DIR}/share/boost-build/boost-modular-build.cmake)
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/b2-options.cmake.in"
    "${CURRENT_BUILDTREES_DIR}/vcpkg-b2-options.cmake"
    @ONLY
)
boost_modular_build(
    SOURCE_PATH ${SOURCE_PATH}
    BOOST_CMAKE_FRAGMENT "${CURRENT_BUILDTREES_DIR}/vcpkg-b2-options.cmake"
)
include(${CURRENT_INSTALLED_DIR}/share/boost-vcpkg-helpers/boost-modular-headers.cmake)
boost_modular_headers(SOURCE_PATH ${SOURCE_PATH})
