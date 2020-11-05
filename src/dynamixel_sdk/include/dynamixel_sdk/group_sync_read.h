/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel Sync Read
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_


#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace dynamixel {

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for reading multiple Dynamixel data from same address with same length at once
////////////////////////////////////////////////////////////////////////////////
    class WINDECLSPEC GroupSyncRead
            {
                    private:
                    PortHandler    *port_;
                    PacketHandler  *ph_;

                    std::vector<uint8_t>            id_list_;
                    std::map<uint8_t, uint8_t *>    data_list_;  // <id, data>
                    std::map<uint8_t, uint8_t *>    error_list_; // <id, error>

                    bool last_result_;
                    bool is_param_changed_;

                    uint8_t        *param_;
                    uint16_t        start_address_;
                    uint16_t        data_length_;

                    void makeParam();

                    public:
                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that Initializes instance for Sync Read
                    /// @param port PortHandler instance
                    /// @param ph PacketHandler instance
                    /// @param start_address Address of the data for read
                    /// @param data_length Length of the data for read
                    ////////////////////////////////////////////////////////////////////////////////
                    GroupSyncRead(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that calls clearParam function to clear the parameter list for Sync Read
                    ////////////////////////////////////////////////////////////////////////////////
                    ~GroupSyncRead() { clearParam(); }

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that returns PortHandler instance
                    /// @return PortHandler instance
                    ////////////////////////////////////////////////////////////////////////////////
                    PortHandler     *getPortHandler()   { return port_; }

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that returns PacketHandler instance
                    /// @return PacketHandler instance
                    ////////////////////////////////////////////////////////////////////////////////
                    PacketHandler   *getPacketHandler() { return ph_; }

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that adds id, start_address, data_length to the Sync Read list
                    /// @param id Dynamixel ID
                    /// @return false
                    /// @return   when the ID exists already in the list
                    /// @return   when the protocol1.0 has been used
                    /// @return or true
                    ////////////////////////////////////////////////////////////////////////////////
                    bool addParam    (uint8_t id);

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that removes id from the Sync Read list
                    /// @param id Dynamixel ID
                    ////////////////////////////////////////////////////////////////////////////////
                    void removeParam (uint8_t id);

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that clears the Sync Read list
                    ////////////////////////////////////////////////////////////////////////////////
                    void clearParam  ();

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that transmits the Sync Read instruction packet which might be constructed by GroupSyncRead::addParam function
                    /// @return COMM_NOT_AVAILABLE
                    /// @return   when the list for Sync Read is empty
                    /// @return   when the protocol1.0 has been used
                    /// @return or the other communication results which come from PacketHandler::syncReadTx
                    ////////////////////////////////////////////////////////////////////////////////
                    int txPacket();

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that receives the packet which might be come from the Dynamixel
                    /// @return COMM_NOT_AVAILABLE
                    /// @return   when the list for Sync Read is empty
                    /// @return   when the protocol1.0 has been used
                    /// @return COMM_SUCCESS
                    /// @return   when there is packet recieved
                    /// @return or the other communication results
                    ////////////////////////////////////////////////////////////////////////////////
                    int rxPacket();

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that transmits and receives the packet which might be come from the Dynamixel
                    /// @return COMM_NOT_AVAILABLE
                    /// @return   when the protocol1.0 has been used
                    /// @return COMM_RX_FAIL
                    /// @return   when there is no packet recieved
                    /// @return COMM_SUCCESS
                    /// @return   when there is packet recieved
                    /// @return or the other communication results which come from GroupBulkRead::txPacket or GroupBulkRead::rxPacket
                    ////////////////////////////////////////////////////////////////////////////////
                    int txRxPacket();

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that checks whether there are available data which might be received by GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
                    /// @param id Dynamixel ID
                    /// @param address Address of the data for read
                    /// @param data_length Length of the data for read
                    /// @return false
                    /// @return   when there are no data available
                    /// @return   when the protocol1.0 has been used
                    /// @return or true
                    ////////////////////////////////////////////////////////////////////////////////
                    bool isAvailable (uint8_t id, uint16_t address, uint16_t data_length);

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that gets the data which might be received by GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
                    /// @param id Dynamixel ID
                    /// @param address Address of the data for read
                    /// @data_length Length of the data for read
                    /// @return data value
                    ////////////////////////////////////////////////////////////////////////////////
                    uint32_t    getData     (uint8_t id, uint16_t address, uint16_t data_length);

                    ////////////////////////////////////////////////////////////////////////////////
                    /// @brief The function that gets the error which might be received by GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
                    /// @param id Dynamixel ID
                    /// @error error of Dynamixel
                    /// @return true
                    /// @return   when Dynamixel returned specific error byte
                    /// @return or false
                    ////////////////////////////////////////////////////////////////////////////////
                    bool getError    (uint8_t id, uint8_t* error);
            };

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_ */
