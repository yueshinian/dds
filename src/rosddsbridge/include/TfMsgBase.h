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
 * @file TfMsgBase.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _TFMSGBASE_H_
#define _TFMSGBASE_H_


#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(TfMsgBase_SOURCE)
#define TfMsgBase_DllAPI __declspec( dllexport )
#else
#define TfMsgBase_DllAPI __declspec( dllimport )
#endif // TfMsgBase_SOURCE
#else
#define TfMsgBase_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define TfMsgBase_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure TfMsgBase defined by the user in the IDL file.
 * @ingroup TFMSGBASE
 */
class TfMsgBase
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport TfMsgBase();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~TfMsgBase();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object TfMsgBase that will be copied.
     */
    eProsima_user_DllExport TfMsgBase(
            const TfMsgBase& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object TfMsgBase that will be copied.
     */
    eProsima_user_DllExport TfMsgBase(
            TfMsgBase&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object TfMsgBase that will be copied.
     */
    eProsima_user_DllExport TfMsgBase& operator =(
            const TfMsgBase& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object TfMsgBase that will be copied.
     */
    eProsima_user_DllExport TfMsgBase& operator =(
            TfMsgBase&& x);

    /*!
     * @brief This function sets a value in member seq
     * @param _seq New value for member seq
     */
    eProsima_user_DllExport void seq(
            uint32_t _seq);

    /*!
     * @brief This function returns the value of member seq
     * @return Value of member seq
     */
    eProsima_user_DllExport uint32_t seq() const;

    /*!
     * @brief This function returns a reference to member seq
     * @return Reference to member seq
     */
    eProsima_user_DllExport uint32_t& seq();

    /*!
     * @brief This function sets a value in member secs
     * @param _secs New value for member secs
     */
    eProsima_user_DllExport void secs(
            uint32_t _secs);

    /*!
     * @brief This function returns the value of member secs
     * @return Value of member secs
     */
    eProsima_user_DllExport uint32_t secs() const;

    /*!
     * @brief This function returns a reference to member secs
     * @return Reference to member secs
     */
    eProsima_user_DllExport uint32_t& secs();

    /*!
     * @brief This function sets a value in member nsecs
     * @param _nsecs New value for member nsecs
     */
    eProsima_user_DllExport void nsecs(
            uint32_t _nsecs);

    /*!
     * @brief This function returns the value of member nsecs
     * @return Value of member nsecs
     */
    eProsima_user_DllExport uint32_t nsecs() const;

    /*!
     * @brief This function returns a reference to member nsecs
     * @return Reference to member nsecs
     */
    eProsima_user_DllExport uint32_t& nsecs();

    /*!
     * @brief This function copies the value in member frame_id
     * @param _frame_id New value to be copied in member frame_id
     */
    eProsima_user_DllExport void frame_id(
            const std::string& _frame_id);

    /*!
     * @brief This function moves the value in member frame_id
     * @param _frame_id New value to be moved in member frame_id
     */
    eProsima_user_DllExport void frame_id(
            std::string&& _frame_id);

    /*!
     * @brief This function returns a constant reference to member frame_id
     * @return Constant reference to member frame_id
     */
    eProsima_user_DllExport const std::string& frame_id() const;

    /*!
     * @brief This function returns a reference to member frame_id
     * @return Reference to member frame_id
     */
    eProsima_user_DllExport std::string& frame_id();
    /*!
     * @brief This function copies the value in member child_frame_id
     * @param _child_frame_id New value to be copied in member child_frame_id
     */
    eProsima_user_DllExport void child_frame_id(
            const std::string& _child_frame_id);

    /*!
     * @brief This function moves the value in member child_frame_id
     * @param _child_frame_id New value to be moved in member child_frame_id
     */
    eProsima_user_DllExport void child_frame_id(
            std::string&& _child_frame_id);

    /*!
     * @brief This function returns a constant reference to member child_frame_id
     * @return Constant reference to member child_frame_id
     */
    eProsima_user_DllExport const std::string& child_frame_id() const;

    /*!
     * @brief This function returns a reference to member child_frame_id
     * @return Reference to member child_frame_id
     */
    eProsima_user_DllExport std::string& child_frame_id();
    /*!
     * @brief This function sets a value in member translation_x
     * @param _translation_x New value for member translation_x
     */
    eProsima_user_DllExport void translation_x(
            float _translation_x);

    /*!
     * @brief This function returns the value of member translation_x
     * @return Value of member translation_x
     */
    eProsima_user_DllExport float translation_x() const;

    /*!
     * @brief This function returns a reference to member translation_x
     * @return Reference to member translation_x
     */
    eProsima_user_DllExport float& translation_x();

    /*!
     * @brief This function sets a value in member translation_y
     * @param _translation_y New value for member translation_y
     */
    eProsima_user_DllExport void translation_y(
            float _translation_y);

    /*!
     * @brief This function returns the value of member translation_y
     * @return Value of member translation_y
     */
    eProsima_user_DllExport float translation_y() const;

    /*!
     * @brief This function returns a reference to member translation_y
     * @return Reference to member translation_y
     */
    eProsima_user_DllExport float& translation_y();

    /*!
     * @brief This function sets a value in member translation_z
     * @param _translation_z New value for member translation_z
     */
    eProsima_user_DllExport void translation_z(
            float _translation_z);

    /*!
     * @brief This function returns the value of member translation_z
     * @return Value of member translation_z
     */
    eProsima_user_DllExport float translation_z() const;

    /*!
     * @brief This function returns a reference to member translation_z
     * @return Reference to member translation_z
     */
    eProsima_user_DllExport float& translation_z();

    /*!
     * @brief This function sets a value in member rotation_x
     * @param _rotation_x New value for member rotation_x
     */
    eProsima_user_DllExport void rotation_x(
            float _rotation_x);

    /*!
     * @brief This function returns the value of member rotation_x
     * @return Value of member rotation_x
     */
    eProsima_user_DllExport float rotation_x() const;

    /*!
     * @brief This function returns a reference to member rotation_x
     * @return Reference to member rotation_x
     */
    eProsima_user_DllExport float& rotation_x();

    /*!
     * @brief This function sets a value in member rotation_y
     * @param _rotation_y New value for member rotation_y
     */
    eProsima_user_DllExport void rotation_y(
            float _rotation_y);

    /*!
     * @brief This function returns the value of member rotation_y
     * @return Value of member rotation_y
     */
    eProsima_user_DllExport float rotation_y() const;

    /*!
     * @brief This function returns a reference to member rotation_y
     * @return Reference to member rotation_y
     */
    eProsima_user_DllExport float& rotation_y();

    /*!
     * @brief This function sets a value in member rotation_z
     * @param _rotation_z New value for member rotation_z
     */
    eProsima_user_DllExport void rotation_z(
            float _rotation_z);

    /*!
     * @brief This function returns the value of member rotation_z
     * @return Value of member rotation_z
     */
    eProsima_user_DllExport float rotation_z() const;

    /*!
     * @brief This function returns a reference to member rotation_z
     * @return Reference to member rotation_z
     */
    eProsima_user_DllExport float& rotation_z();

    /*!
     * @brief This function sets a value in member rotation_w
     * @param _rotation_w New value for member rotation_w
     */
    eProsima_user_DllExport void rotation_w(
            float _rotation_w);

    /*!
     * @brief This function returns the value of member rotation_w
     * @return Value of member rotation_w
     */
    eProsima_user_DllExport float rotation_w() const;

    /*!
     * @brief This function returns a reference to member rotation_w
     * @return Reference to member rotation_w
     */
    eProsima_user_DllExport float& rotation_w();


    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(
            const TfMsgBase& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(
            eprosima::fastcdr::Cdr& cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const;

private:

    uint32_t m_seq;
    uint32_t m_secs;
    uint32_t m_nsecs;
    std::string m_frame_id;
    std::string m_child_frame_id;
    float m_translation_x;
    float m_translation_y;
    float m_translation_z;
    float m_rotation_x;
    float m_rotation_y;
    float m_rotation_z;
    float m_rotation_w;
};

#endif // _TFMSGBASE_H_