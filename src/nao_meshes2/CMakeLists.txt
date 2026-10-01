cmake_minimum_required(VERSION 3.18)
project(nao_meshes)

find_package(ament_cmake REQUIRED)

# When getting a new binay, the MD5 might change, and the folder hierarchy too
# read comments below to adapt

set(TMP_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(INSTALLER_PATH ${TMP_DIR}/installer.run)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "$i.86$")
  message(STATUS "Downloading 32-bit installer")
  set(INSTALLER_NAME "naomeshes-0.6.7-linux-installer.run")
  set(INSTALLER_MD5 1984b2790930044b4ee00496315c0021)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  message(STATUS "Downloading 64-bit installer")
  set(INSTALLER_NAME "naomeshes-0.6.7-linux-x64-installer.run")
  set(INSTALLER_MD5 5f897a3328f217bae6c0ed2a934e524c)
else()
  message(FATAL_ERROR "Mesh installer is not compatible with your host architecture '${CMAKE_HOST_SYSTEM_PROCESSOR}'")
endif()

# The installer is stored in the nao_meshes_installer repository. Its raw file
# is served by raw.githubusercontent.com, which is not reachable from every
# network. The same content is also available as a repository tarball from
# codeload.github.com, so download that and extract the installer from it.
set(TARBALL_URL "https://codeload.github.com/ros-naoqi/nao_meshes_installer/tar.gz/refs/heads/master")
set(TARBALL_PATH ${TMP_DIR}/installer_repo.tar.gz)
set(EXTRACT_DIR ${TMP_DIR}/installer_repo)

set(_tarball_downloaded FALSE)
foreach(_attempt RANGE 1 10)
  file(DOWNLOAD ${TARBALL_URL} ${TARBALL_PATH}
    SHOW_PROGRESS
    INACTIVITY_TIMEOUT 30
    STATUS _download_status
  )
  list(GET _download_status 0 _download_status_code)
  if(_download_status_code EQUAL 0)
    set(_tarball_downloaded TRUE)
    break()
  endif()
  list(GET _download_status 1 _download_error)
  message(STATUS "Download attempt ${_attempt}/10 failed: ${_download_error}; retrying...")
  file(REMOVE ${TARBALL_PATH})
endforeach()

if(NOT _tarball_downloaded)
  message(FATAL_ERROR "Failed to download the meshes installer tarball from '${TARBALL_URL}'")
endif()

file(ARCHIVE_EXTRACT INPUT ${TARBALL_PATH} DESTINATION ${EXTRACT_DIR})

file(GLOB_RECURSE _installer_candidates ${EXTRACT_DIR}/${INSTALLER_NAME})
list(LENGTH _installer_candidates _installer_candidates_len)
if(_installer_candidates_len EQUAL 0)
  message(FATAL_ERROR "Could not find '${INSTALLER_NAME}' in the downloaded tarball")
endif()
list(GET _installer_candidates 0 _installer_file)

file(MD5 ${_installer_file} _actual_md5)
if(NOT _actual_md5 STREQUAL INSTALLER_MD5)
  message(FATAL_ERROR "Installer MD5 mismatch: expected '${INSTALLER_MD5}', got '${_actual_md5}'")
endif()
file(COPY ${_installer_file} DESTINATION ${TMP_DIR})
file(RENAME ${TMP_DIR}/${INSTALLER_NAME} ${INSTALLER_PATH})

# specify the interesting folder in the archive. You might want to change
# that for a new binary where the structure is different
set(TMP_MESH_PATH ${TMP_DIR}/meshes)
set(TMP_TEXTURE_PATH ${TMP_DIR}/texture)

# Where to install the meshes and textures
# relative to the share folder of the package.
set(INSTALL_DESTINATION_IN_SHARE ${PROJECT_NAME})

# install the meshes in the build directory when building
message(STATUS "Unpacking meshes from the installer: \n${INSTALLER_PATH} --prefix ${TMP_DIR}")
add_custom_target(install_meshes ALL
  COMMAND chmod +x ${INSTALLER_PATH}
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/install_with_auto_mode.sh ${INSTALLER_PATH} --prefix ${TMP_DIR}
  BYPRODUCTS ${TMP_MESH_PATH} ${TMP_TEXTURE_PATH}
  TERMINAL
)

# install the archive
install(DIRECTORY ${TMP_MESH_PATH} ${TMP_TEXTURE_PATH}
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
