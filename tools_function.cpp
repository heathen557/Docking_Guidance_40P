//
// Created by luffy7n on 18-11-10.
//

#include "tools_function.h"


//hash_t hash_(char const* str)
//{
//    hash_t ret{basis};
//
//    while(*str){
//        ret ^= *str;
//        ret *= prime;
//        str++;
//    }
//
//    return ret;
//}
//
//constexpr hash_t hash_compile_time(char const* str, hash_t last_value = basis)
//{
//    return *str ? hash_compile_time(str+1, (*str ^ last_value) * prime) : last_value;
//}

POSINFO Plane_Straight_line_LR(const Input_Num &num1, const Input_Num &num2, Input_Num Rec_values)
{
    float Y;
    float K;
    float b;
    POSINFO status;
    std::string R("RIGHT");
    std::string L("LEFT");
    std::string M("MILD");

    K=(num1.y-num2.y)/(num1.x-num2.x);
    b=(num1.x*num2.y-num1.y*num2.x)/(num1.x-num2.x);

    float Dis = fabs((K * Rec_values.x - Rec_values.y + b)/sqrt(K * K + 1));
    if (Dis <= 0.1)
    {
        //std::cout<<"ON PATH"<<std::endl;
        status.position = M;
        status.offset = 0.0;
        return status;
    }
    else
    {
        //std::cout << "NOT ON PATH" << std::endl;
        if (Rec_values.x > ((Rec_values.y-b)/K))
        {
            //offset_ =
            //std::cout << "L" << std::endl;
            status.position = R;
            status.offset = Dis;
            return status;
        }
        else
        {
            //offset =
            status.position = L;
            status.offset = Dis;
            return status;
            // std::cout << "R" <<std::endl;
        }

        /*if (Rec_values.y > K * Rec_values.x + b)
        {
            status.offset = Dis;
        }
        else
        {
            status.offset = -Dis;
        }*/
    }

}

POSINFO Plane_Straight_line_UD(bool direction, Input_Num &num1, const Input_Num &num2, Input_Num Rec_values)
{
    float K;
    float b;
    POSINFO status;
    std::string U("UP");
    std::string D("DOWN");
    std::string M("MILD");

    K = (num1.y - num2.y) / (num1.x - num2.x);
    b = (num1.x * num2.y - num1.y * num2.x) / (num1.x - num2.x);

    float Dis = fabs((K * Rec_values.x - Rec_values.y + b) / sqrt(K * K + 1));
    if (Dis <= 0.1)
    {
        //std::cout<<"ON PATH"<<std::endl;
        status.position = M;
        status.offset = 0.0;
        return status;
    }
    else
    {
        if (direction) {
            //std::cout << "NOT ON PATH" << std::endl;
            if (Rec_values.y > K * Rec_values.x + b) {
                //offset_ =
                //std::cout << "L" << std::endl;
                status.position = U;
                status.offset = Dis;
                return status;
            } else {
                //offset =
                status.position = D;
                status.offset = -Dis;
                return status;
                // std::cout << "R" <<std::endl;
            }
        } else {
            if (Rec_values.y > K * Rec_values.x + b) {
                //offset_ =
                //std::cout << "L" << std::endl;
                status.position = D;
                status.offset = -Dis;
                return status;
            } else {
                //offset =
                status.position = U;
                status.offset = Dis;
                return status;
                // std::cout << "R" <<std::endl;
            }
        }
    }
}

float minkowsky(const std::vector<float > &v1, const std::vector<float> &v2, float m)
{
    assert(v1.size() == v2.size());
    float distance = 0.0;
    for (size_t i = 0; i < v1.size(); ++i)
        distance += pow(fabsf(v1[i] - v2[i]), m);
    return pow(distance, 1.0 / m);
}

float calculateSimilarity(const std::vector<float> &v1, const std::vector<float> &v2)
{
    assert(v1.size() == v2.size());
    return minkowsky(v1, v2, 2.0);
}

float calculateVelocity(vector<float> &p)
{
    float add_num = 0;
    for (int i = 1; i < p.size(); ++i)
    {
        float displacement = p[i] - p[i-1];
        add_num += displacement;
    }
    float velocity = add_num / 0.8; // 0.1 * 4 　0.1是根据扫描频率确定的
    return velocity;
}

float calculateLength(std::vector<float> &length)
{
    float sum_length = 0.0;
    int size = length.size();
    sort(length.begin(), length.end());
    for (int i = 1; i < 6; ++i)
    {
        sum_length += length[size-i];
    }
    float statistics_length = sum_length / 5;
    return statistics_length;
}

void controlKey(int frame)
{
    char key;
    do
    {
        key = static_cast<char>(getchar());
        printf("%d\n", frame);
    }while (key != 27 && key != 'q' && key != 'Q');
}