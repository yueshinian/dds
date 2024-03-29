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
 * @file VelodyneWithOdometryMsg.cpp
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

#include "VelodyneWithOdometryMsg.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

VelodyneWithOdometryMsg::VelodyneWithOdometryMsg()
{
    // m_odometry com.eprosima.fastdds.idl.parser.typecode.StructTypeCode@71248c21

    // m_pcl com.eprosima.fastdds.idl.parser.typecode.StructTypeCode@49e202ad


}

VelodyneWithOdometryMsg::~VelodyneWithOdometryMsg()
{


}

VelodyneWithOdometryMsg::VelodyneWithOdometryMsg(
        const VelodyneWithOdometryMsg& x)
{
    m_odometry = x.m_odometry;
    m_pcl = x.m_pcl;
}

VelodyneWithOdometryMsg::VelodyneWithOdometryMsg(
        VelodyneWithOdometryMsg&& x)
{
    m_odometry = std::move(x.m_odometry);
    m_pcl = std::move(x.m_pcl);
}

VelodyneWithOdometryMsg& VelodyneWithOdometryMsg::operator =(
        const VelodyneWithOdometryMsg& x)
{

    m_odometry = x.m_odometry;
    m_pcl = x.m_pcl;

    return *this;
}

VelodyneWithOdometryMsg& VelodyneWithOdometryMsg::operator =(
        VelodyneWithOdometryMsg&& x)
{

    m_odometry = std::move(x.m_odometry);
    m_pcl = std::move(x.m_pcl);

    return *this;
}

size_t VelodyneWithOdometryMsg::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += OdometryMsg::getMaxCdrSerializedSize(current_alignment);
    current_alignment += PclMsg::getMaxCdrSerializedSize(current_alignment);

    return current_alignment - initial_alignment;
}

size_t VelodyneWithOdometryMsg::getCdrSerializedSize(
        const VelodyneWithOdometryMsg& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += OdometryMsg::getCdrSerializedSize(data.odometry(), current_alignment);
    current_alignment += PclMsg::getCdrSerializedSize(data.pcl(), current_alignment);

    return current_alignment - initial_alignment;
}

void VelodyneWithOdometryMsg::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_odometry;
    scdr << m_pcl;

}

void VelodyneWithOdometryMsg::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_odometry;
    dcdr >> m_pcl;
}

/*!
 * @brief This function copies the value in member odometry
 * @param _odometry New value to be copied in member odometry
 */
void VelodyneWithOdometryMsg::odometry(
        const OdometryMsg& _odometry)
{
    m_odometry = _odometry;
}

/*!
 * @brief This function moves the value in member odometry
 * @param _odometry New value to be moved in member odometry
 */
void VelodyneWithOdometryMsg::odometry(
        OdometryMsg&& _odometry)
{
    m_odometry = std::move(_odometry);
}

/*!
 * @brief This function returns a constant reference to member odometry
 * @return Constant reference to member odometry
 */
const OdometryMsg& VelodyneWithOdometryMsg::odometry() const
{
    return m_odometry;
}

/*!
 * @brief This function returns a reference to member odometry
 * @return Reference to member odometry
 */
OdometryMsg& VelodyneWithOdometryMsg::odometry()
{
    return m_odometry;
}
/*!
 * @brief This function copies the value in member pcl
 * @param _pcl New value to be copied in member pcl
 */
void VelodyneWithOdometryMsg::pcl(
        const PclMsg& _pcl)
{
    m_pcl = _pcl;
}

/*!
 * @brief This function moves the value in member pcl
 * @param _pcl New value to be moved in member pcl
 */
void VelodyneWithOdometryMsg::pcl(
        PclMsg&& _pcl)
{
    m_pcl = std::move(_pcl);
}

/*!
 * @brief This function returns a constant reference to member pcl
 * @return Constant reference to member pcl
 */
const PclMsg& VelodyneWithOdometryMsg::pcl() const
{
    return m_pcl;
}

/*!
 * @brief This function returns a reference to member pcl
 * @return Reference to member pcl
 */
PclMsg& VelodyneWithOdometryMsg::pcl()
{
    return m_pcl;
}

size_t VelodyneWithOdometryMsg::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;





    return current_align;
}

bool VelodyneWithOdometryMsg::isKeyDefined()
{
    return false;
}

void VelodyneWithOdometryMsg::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
      
}
