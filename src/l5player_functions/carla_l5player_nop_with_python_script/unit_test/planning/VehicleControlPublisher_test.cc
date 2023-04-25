#include "gtest/gtest.h"
#include "../../src/planning/reference_line/reference_line.h"

TEST(ClientHandler, sendParams)
{
    // 测试的时候的交互方式也不能改变，既然client实际的效果是在命令行输入参数,
    // 那这里也是这样的效果
    l5player::reference_line::referenceLine rl_;

    EXPECT_TRUE(1);
}