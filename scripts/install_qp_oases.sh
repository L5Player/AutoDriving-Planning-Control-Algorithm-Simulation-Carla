# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

 
git clone -b releases/3.2.1 --depth=1 https://gitee.com/luyi5894/qpOASES.git

cd qpOASES/
mkdir bin
make -j8 CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion \
                   -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE \
                   -D__NO_COPYRIGHT__"
cp bin/libqpOASES.so /usr/local/lib
cp -r include/* /usr/local/include
cd ..

# Clean up.
rm -fr qpOASES
