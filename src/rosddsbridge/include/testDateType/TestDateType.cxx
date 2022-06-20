// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file TestDateType.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "TestDateType.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

TestDateType::TestDateType()
{
    // m_a com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5204062d
    m_a = 0;
    // m_b com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4fcd19b3
    m_b = 0;
    // m_c com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5d11346a
    m_c = 0;
    // m_d com.eprosima.idl.parser.typecode.PrimitiveTypeCode@b3d7190
    m_d = 0;
    // m_e com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5fdba6f9
    m_e = 0;
    // m_f com.eprosima.idl.parser.typecode.PrimitiveTypeCode@10d59286
    m_f = 0;
    // m_g com.eprosima.idl.parser.typecode.PrimitiveTypeCode@fe18270
    m_g = 0;
    // m_h com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6fb0d3ed
    m_h = 0;
    // m_i com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6dde5c8c
    m_i = 0;
    // m_j com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5123a213
    m_j = 0.0;
    // m_k com.eprosima.idl.parser.typecode.PrimitiveTypeCode@52525845
    m_k = 0.0;
    // m_l com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3b94d659
    m_l = 0.0;
    // m_m com.eprosima.idl.parser.typecode.PrimitiveTypeCode@24b1d79b
    m_m = false;
    // m_n com.eprosima.idl.parser.typecode.SequenceTypeCode@68ceda24


}

TestDateType::~TestDateType()
{














}

TestDateType::TestDateType(
        const TestDateType& x)
{
    m_a = x.m_a;
    m_b = x.m_b;
    m_c = x.m_c;
    m_d = x.m_d;
    m_e = x.m_e;
    m_f = x.m_f;
    m_g = x.m_g;
    m_h = x.m_h;
    m_i = x.m_i;
    m_j = x.m_j;
    m_k = x.m_k;
    m_l = x.m_l;
    m_m = x.m_m;
    m_n = x.m_n;
}

TestDateType::TestDateType(
        TestDateType&& x)
{
    m_a = x.m_a;
    m_b = x.m_b;
    m_c = x.m_c;
    m_d = x.m_d;
    m_e = x.m_e;
    m_f = x.m_f;
    m_g = x.m_g;
    m_h = x.m_h;
    m_i = x.m_i;
    m_j = x.m_j;
    m_k = x.m_k;
    m_l = x.m_l;
    m_m = x.m_m;
    m_n = std::move(x.m_n);
}

TestDateType& TestDateType::operator =(
        const TestDateType& x)
{

    m_a = x.m_a;
    m_b = x.m_b;
    m_c = x.m_c;
    m_d = x.m_d;
    m_e = x.m_e;
    m_f = x.m_f;
    m_g = x.m_g;
    m_h = x.m_h;
    m_i = x.m_i;
    m_j = x.m_j;
    m_k = x.m_k;
    m_l = x.m_l;
    m_m = x.m_m;
    m_n = x.m_n;

    return *this;
}

TestDateType& TestDateType::operator =(
        TestDateType&& x)
{

    m_a = x.m_a;
    m_b = x.m_b;
    m_c = x.m_c;
    m_d = x.m_d;
    m_e = x.m_e;
    m_f = x.m_f;
    m_g = x.m_g;
    m_h = x.m_h;
    m_i = x.m_i;
    m_j = x.m_j;
    m_k = x.m_k;
    m_l = x.m_l;
    m_m = x.m_m;
    m_n = std::move(x.m_n);

    return *this;
}

size_t TestDateType::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);




    return current_alignment - initial_alignment;
}

size_t TestDateType::getCdrSerializedSize(
        const TestDateType& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.n().size() > 0)
    {
        current_alignment += (data.n().size() * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }




    return current_alignment - initial_alignment;
}

void TestDateType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_a;
    scdr << m_b;
    scdr << m_c;
    scdr << m_d;
    scdr << m_e;
    scdr << m_f;
    scdr << m_g;
    scdr << m_h;
    scdr << m_i;
    scdr << m_j;
    scdr << m_k;
    scdr << m_l;
    scdr << m_m;
    scdr << m_n;

}

void TestDateType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_a;
    dcdr >> m_b;
    dcdr >> m_c;
    dcdr >> m_d;
    dcdr >> m_e;
    dcdr >> m_f;
    dcdr >> m_g;
    dcdr >> m_h;
    dcdr >> m_i;
    dcdr >> m_j;
    dcdr >> m_k;
    dcdr >> m_l;
    dcdr >> m_m;
    dcdr >> m_n;
}

/*!
 * @brief This function sets a value in member a
 * @param _a New value for member a
 */
void TestDateType::a(
        uint8_t _a)
{
    m_a = _a;
}

/*!
 * @brief This function returns the value of member a
 * @return Value of member a
 */
uint8_t TestDateType::a() const
{
    return m_a;
}

/*!
 * @brief This function returns a reference to member a
 * @return Reference to member a
 */
uint8_t& TestDateType::a()
{
    return m_a;
}

/*!
 * @brief This function sets a value in member b
 * @param _b New value for member b
 */
void TestDateType::b(
        char _b)
{
    m_b = _b;
}

/*!
 * @brief This function returns the value of member b
 * @return Value of member b
 */
char TestDateType::b() const
{
    return m_b;
}

/*!
 * @brief This function returns a reference to member b
 * @return Reference to member b
 */
char& TestDateType::b()
{
    return m_b;
}

/*!
 * @brief This function sets a value in member c
 * @param _c New value for member c
 */
void TestDateType::c(
        wchar_t _c)
{
    m_c = _c;
}

/*!
 * @brief This function returns the value of member c
 * @return Value of member c
 */
wchar_t TestDateType::c() const
{
    return m_c;
}

/*!
 * @brief This function returns a reference to member c
 * @return Reference to member c
 */
wchar_t& TestDateType::c()
{
    return m_c;
}

/*!
 * @brief This function sets a value in member d
 * @param _d New value for member d
 */
void TestDateType::d(
        int16_t _d)
{
    m_d = _d;
}

/*!
 * @brief This function returns the value of member d
 * @return Value of member d
 */
int16_t TestDateType::d() const
{
    return m_d;
}

/*!
 * @brief This function returns a reference to member d
 * @return Reference to member d
 */
int16_t& TestDateType::d()
{
    return m_d;
}

/*!
 * @brief This function sets a value in member e
 * @param _e New value for member e
 */
void TestDateType::e(
        uint16_t _e)
{
    m_e = _e;
}

/*!
 * @brief This function returns the value of member e
 * @return Value of member e
 */
uint16_t TestDateType::e() const
{
    return m_e;
}

/*!
 * @brief This function returns a reference to member e
 * @return Reference to member e
 */
uint16_t& TestDateType::e()
{
    return m_e;
}

/*!
 * @brief This function sets a value in member f
 * @param _f New value for member f
 */
void TestDateType::f(
        int32_t _f)
{
    m_f = _f;
}

/*!
 * @brief This function returns the value of member f
 * @return Value of member f
 */
int32_t TestDateType::f() const
{
    return m_f;
}

/*!
 * @brief This function returns a reference to member f
 * @return Reference to member f
 */
int32_t& TestDateType::f()
{
    return m_f;
}

/*!
 * @brief This function sets a value in member g
 * @param _g New value for member g
 */
void TestDateType::g(
        uint32_t _g)
{
    m_g = _g;
}

/*!
 * @brief This function returns the value of member g
 * @return Value of member g
 */
uint32_t TestDateType::g() const
{
    return m_g;
}

/*!
 * @brief This function returns a reference to member g
 * @return Reference to member g
 */
uint32_t& TestDateType::g()
{
    return m_g;
}

/*!
 * @brief This function sets a value in member h
 * @param _h New value for member h
 */
void TestDateType::h(
        int64_t _h)
{
    m_h = _h;
}

/*!
 * @brief This function returns the value of member h
 * @return Value of member h
 */
int64_t TestDateType::h() const
{
    return m_h;
}

/*!
 * @brief This function returns a reference to member h
 * @return Reference to member h
 */
int64_t& TestDateType::h()
{
    return m_h;
}

/*!
 * @brief This function sets a value in member i
 * @param _i New value for member i
 */
void TestDateType::i(
        uint64_t _i)
{
    m_i = _i;
}

/*!
 * @brief This function returns the value of member i
 * @return Value of member i
 */
uint64_t TestDateType::i() const
{
    return m_i;
}

/*!
 * @brief This function returns a reference to member i
 * @return Reference to member i
 */
uint64_t& TestDateType::i()
{
    return m_i;
}

/*!
 * @brief This function sets a value in member j
 * @param _j New value for member j
 */
void TestDateType::j(
        float _j)
{
    m_j = _j;
}

/*!
 * @brief This function returns the value of member j
 * @return Value of member j
 */
float TestDateType::j() const
{
    return m_j;
}

/*!
 * @brief This function returns a reference to member j
 * @return Reference to member j
 */
float& TestDateType::j()
{
    return m_j;
}

/*!
 * @brief This function sets a value in member k
 * @param _k New value for member k
 */
void TestDateType::k(
        double _k)
{
    m_k = _k;
}

/*!
 * @brief This function returns the value of member k
 * @return Value of member k
 */
double TestDateType::k() const
{
    return m_k;
}

/*!
 * @brief This function returns a reference to member k
 * @return Reference to member k
 */
double& TestDateType::k()
{
    return m_k;
}

/*!
 * @brief This function sets a value in member l
 * @param _l New value for member l
 */
void TestDateType::l(
        long double _l)
{
    m_l = _l;
}

/*!
 * @brief This function returns the value of member l
 * @return Value of member l
 */
long double TestDateType::l() const
{
    return m_l;
}

/*!
 * @brief This function returns a reference to member l
 * @return Reference to member l
 */
long double& TestDateType::l()
{
    return m_l;
}

/*!
 * @brief This function sets a value in member m
 * @param _m New value for member m
 */
void TestDateType::m(
        bool _m)
{
    m_m = _m;
}

/*!
 * @brief This function returns the value of member m
 * @return Value of member m
 */
bool TestDateType::m() const
{
    return m_m;
}

/*!
 * @brief This function returns a reference to member m
 * @return Reference to member m
 */
bool& TestDateType::m()
{
    return m_m;
}

/*!
 * @brief This function copies the value in member n
 * @param _n New value to be copied in member n
 */
void TestDateType::n(
        const std::vector<int32_t>& _n)
{
    m_n = _n;
}

/*!
 * @brief This function moves the value in member n
 * @param _n New value to be moved in member n
 */
void TestDateType::n(
        std::vector<int32_t>&& _n)
{
    m_n = std::move(_n);
}

/*!
 * @brief This function returns a constant reference to member n
 * @return Constant reference to member n
 */
const std::vector<int32_t>& TestDateType::n() const
{
    return m_n;
}

/*!
 * @brief This function returns a reference to member n
 * @return Reference to member n
 */
std::vector<int32_t>& TestDateType::n()
{
    return m_n;
}

size_t TestDateType::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;

















    return current_align;
}

bool TestDateType::isKeyDefined()
{
    return false;
}

void TestDateType::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
                  
}