#!/bin/sh

usage() {
    echo "usage:"
    echo "${0} <sdk-archive>"
    echo ""
    echo " <sdk-achive> : The achive file with the aldebaran linux sdk"
    exit 1
}

if [ $# -ne 1 ]
then 
    usage
else
    archive="${1}"
fi

unzip ${archive}
name=${archive%.*}
mv ${name} ctc
cd ctc

sh ./yocto-sdk/relocate_qitoolchain.sh
