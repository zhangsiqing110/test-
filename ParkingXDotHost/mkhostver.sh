#!/bin/bash

IAR_CFG=$@
#BOARD_DIR=$(realpath $(dirname $0))
#SYM_DIR=${BOARD_DIR}/sym
SDK_DIR=$(realpath $(dirname $0))
PNI_VERSION_DIR=${SDK_DIR}/pni_version
VERSION_FILE=${PNI_VERSION_DIR}/version.txt
BUILD_NUMBER_FILE=${PNI_VERSION_DIR}/build_number.txt
CRC_FILE=${PNI_VERSION_DIR}/crc.txt
PROJECT_FILE=${PNI_VERSION_DIR}/project.txt
DIRTY_BUILD=false
AUTO_VERSION_TEMP=${PNI_VERSION_DIR}/auto_version_template.h
AUTO_VERSION=${SDK_DIR}/Inc/autogen_version.h
IAR_PROJ=EWARM/${IAR_CFG}.ewp
IAR_PROJECT_NAME=$1
IAR_BUILT_DIR=${SDK_DIR}/EWARM/${IAR_PROJECT_NAME}/Exe
IAR_BUILT_IMAGE=${IAR_PROJECT_NAME}
IAR_BUILT_BIN_IMAGE=${IAR_BUILT_DIR}/${IAR_BUILT_IMAGE}.bin

cd ${SDK_DIR}

if [ -z ${IAR_CFG} ]; then
	echo
	echo "### Error: Must provide IAR config name (FRT_Demo/Tethered_SENtrace)"
	echo
	exit 1
fi

if [ -x "$(command -v IarBuild.exe)" ]; then
IarBuild.exe ${IAR_PROJ} -clean ${IAR_CFG}
fi
#make clean
#cd boards


# check version file exists
if [ ! -f ${VERSION_FILE} ]; then
	echo
	echo "### Error: couldn't locate version file: ${VERSION_FILE}"
	echo
	exit 1
fi

if [ ! -f ${BUILD_NUMBER_FILE} ]; then
	echo
	echo "### Error: couldn't locate build number file: ${BUILD_NUMBER_FILE}"
	echo
	exit 1
fi

if [ ! -f ${PROJECT_FILE} ]; then
	echo
	echo "### Error: couldn't locate project file: ${PROJECT_FILE}"
	echo
	exit 1
fi

# try get version tag from git
#GIT_VER=$(git tag --contains)
#GIT_VER=$(git describe --exact-match --tags)
#GIT_VER=SENTRACE_01_02_03_004

#GIT_VER_PROJ=$(echo $GIT_VER | cut -f1 -d_)
#GIT_VER_MAJOR=$(echo $GIT_VER | cut -f2 -d_)
#GIT_VER_MINOR=$(echo $GIT_VER | cut -f3 -d_)
#GIT_VER_PATCH=$(echo $GIT_VER | cut -f4 -d_)
#GIT_VER_OTHER=$(echo $GIT_VER | cut -f5 -d_)
#GIT_VER_BUILD=$(git rev-parse --short HEAD)

#echo "GIT Version Tags: ${GIT_VER_MAJOR}.${GIT_VER_MINOR}.${GIT_VER_PATCH}.${GIT_VER_OTHER}.${GIT_VER_BUILD}"

# if git version tag not found, then use version.txt
IFS=. read VERSION_MAJOR VERSION_MINOR VERSION_PATCH < "${VERSION_FILE}"
read VERSION_BUILD_NUMBER < ${BUILD_NUMBER_FILE}
read PROJECT_NAME < ${PROJECT_FILE}


if [ -n "$(git status --column --porcelain)" ]; then
	echo "Found unsaved files, DIRTY BUILD!!"
	DIRTY_BUILD=true
	VERSION_BUILD_NUMBER=`curl -s "http://emb3.pnicorp.com/app_build_number.php?name=${PROJECT_NAME}"`
	echo ${VERSION_BUILD_NUMBER} > ${BUILD_NUMBER_FILE}

	# make we also update the autogen file
	cp -f "$AUTO_VERSION_TEMP" "${AUTO_VERSION}"
	echo "#ifndef PNI_AUTO_VERSION_H" >> "${AUTO_VERSION}"
	echo "#define PNI_AUTO_VERSION_H" >> "${AUTO_VERSION}"
	echo "#define HOST_REL_MAJOR  " ${VERSION_MAJOR} >> "${AUTO_VERSION}"
	echo "#define HOST_REL_MINOR  " ${VERSION_MINOR} >> "${AUTO_VERSION}"
	echo "#define HOST_REL_PATCH  " ${VERSION_PATCH} >> "${AUTO_VERSION}"
	echo "#define HOST_REL_OTHER  " ${VERSION_OTHER} >> "${AUTO_VERSION}"
	echo "#define HOST_REL_BUILD   " ${VERSION_BUILD_NUMBER} >> "${AUTO_VERSION}"
	echo "#endif" >> "${AUTO_VERSION}"
fi

if [ -x "$(command -v IarBuild.exe)"  ]; then
IarBuild.exe ${IAR_PROJ} -build ${IAR_CFG}
fi

if [ "${DIRTY_BUILD}" = true ] && [ -z "$(git diff ${VERSION_FILE})" ]; then
	echo
	echo "### Warning: found changes to tracked files but version file has not been changed. Do you need to increment the version?"
	echo
fi

echo "Version: ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.${VERSION_BUILD_NUMBER}"

if [ -f ${IAR_BUILT_BIN_IMAGE}  ]; then
	echo
	echo "### duplicate ${IAR_BUILT_IMAGE}.bin with version"
	echo
	cp ${IAR_BUILT_BIN_IMAGE} ${IAR_BUILT_DIR}/${IAR_BUILT_IMAGE}_v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.${VERSION_BUILD_NUMBER}.bin
fi

if [ "${DIRTY_BUILD}" = true ]; then
	rm -f ${CRC_FILE}
fi

#for fw_file in ${SYM_DIR}/*.fw
#do
#	FW_CRC="0x$(od -An -tx4 -j4 -N4 ${fw_file} | tr -d '[[:space:]]' | awk '{print toupper($0)}') -- $(basename ${fw_file})"
#	echo ${FW_CRC}
#	if [ "${DIRTY_BUILD}" = true ]; then
#		echo ${FW_CRC} >> ${CRC_FILE}
#	fi
#done



