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
 * @file ImageMsg.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _IMAGEMSG_H_
#define _IMAGEMSG_H_


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
#if defined(ImageMsg_SOURCE)
#define ImageMsg_DllAPI __declspec( dllexport )
#else
#define ImageMsg_DllAPI __declspec( dllimport )
#endif // ImageMsg_SOURCE
#else
#define ImageMsg_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define ImageMsg_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure ImageMsg defined by the user in the IDL file.
 * @ingroup IMAGEMSG
 */
class ImageMsg
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport ImageMsg();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~ImageMsg();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object ImageMsg that will be copied.
     */
    eProsima_user_DllExport ImageMsg(
            const ImageMsg& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object ImageMsg that will be copied.
     */
    eProsima_user_DllExport ImageMsg(
            ImageMsg&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object ImageMsg that will be copied.
     */
    eProsima_user_DllExport ImageMsg& operator =(
            const ImageMsg& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object ImageMsg that will be copied.
     */
    eProsima_user_DllExport ImageMsg& operator =(
            ImageMsg&& x);

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
            double _secs);

    /*!
     * @brief This function returns the value of member secs
     * @return Value of member secs
     */
    eProsima_user_DllExport double secs() const;

    /*!
     * @brief This function returns a reference to member secs
     * @return Reference to member secs
     */
    eProsima_user_DllExport double& secs();

    /*!
     * @brief This function sets a value in member nsecs
     * @param _nsecs New value for member nsecs
     */
    eProsima_user_DllExport void nsecs(
            float _nsecs);

    /*!
     * @brief This function returns the value of member nsecs
     * @return Value of member nsecs
     */
    eProsima_user_DllExport float nsecs() const;

    /*!
     * @brief This function returns a reference to member nsecs
     * @return Reference to member nsecs
     */
    eProsima_user_DllExport float& nsecs();

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
     * @brief This function copies the value in member format
     * @param _format New value to be copied in member format
     */
    eProsima_user_DllExport void format(
            const std::string& _format);

    /*!
     * @brief This function moves the value in member format
     * @param _format New value to be moved in member format
     */
    eProsima_user_DllExport void format(
            std::string&& _format);

    /*!
     * @brief This function returns a constant reference to member format
     * @return Constant reference to member format
     */
    eProsima_user_DllExport const std::string& format() const;

    /*!
     * @brief This function returns a reference to member format
     * @return Reference to member format
     */
    eProsima_user_DllExport std::string& format();
    /*!
     * @brief This function sets a value in member datacount
     * @param _datacount New value for member datacount
     */
    eProsima_user_DllExport void datacount(
            uint32_t _datacount);

    /*!
     * @brief This function returns the value of member datacount
     * @return Value of member datacount
     */
    eProsima_user_DllExport uint32_t datacount() const;

    /*!
     * @brief This function returns a reference to member datacount
     * @return Reference to member datacount
     */
    eProsima_user_DllExport uint32_t& datacount();

    /*!
     * @brief This function copies the value in member data
     * @param _data New value to be copied in member data
     */
    eProsima_user_DllExport void data(
            const std::vector<char>& _data);

    /*!
     * @brief This function moves the value in member data
     * @param _data New value to be moved in member data
     */
    eProsima_user_DllExport void data(
            std::vector<char>&& _data);

    /*!
     * @brief This function returns a constant reference to member data
     * @return Constant reference to member data
     */
    eProsima_user_DllExport const std::vector<char>& data() const;

    /*!
     * @brief This function returns a reference to member data
     * @return Reference to member data
     */
    eProsima_user_DllExport std::vector<char>& data();

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
            const ImageMsg& data,
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
    double m_secs;
    float m_nsecs;
    std::string m_frame_id;
    std::string m_format;
    uint32_t m_datacount;
    std::vector<char> m_data;
};

#endif // _IMAGEMSG_H_