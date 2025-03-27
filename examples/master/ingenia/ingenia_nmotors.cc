/*
NOTE: This ingenia example works with multiple slaves in the bus, 
*/

#include <iostream>
#include <cstring>

#include "kickcat/Bus.h"
#include "kickcat/Link.h"
#include "kickcat/Prints.h"
#include "kickcat/SocketNull.h"
#include "kickcat/helpers.h"

#include "CanOpenErrors.h"
#include "CanOpenStateMachine.h"
#include "IngeniaProtocol.h"

#ifdef __linux__
    #include "kickcat/OS/Linux/Socket.h"
#elif __PikeOS__
    #include "kickcat/OS/PikeOS/Socket.h"
#elif __MINGW64__
    #include "kickcat/OS/Windows/Socket.h"
#else
#error "Unknown platform"
#endif


using namespace kickcat;

int printErrorCode(kickcat::ErrorCode const& e)
{
    std::cerr << e.what() << ": " << ALStatus_to_string(e.code()) << std::endl;
    return 1;
};

int printException(std::exception const& e)
{
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
};


int main(int argc, char *argv[])
{
    if (argc != 3 and argc != 2)
    {
        printf("usage redundancy mode : ./test NIC_nominal NIC_redundancy\n");
        printf("usage no redundancy mode : ./test NIC_nominal\n");
        return 1;
    }

    std::shared_ptr<AbstractSocket> socket_redundancy;
    std::string red_interface_name = "null";
    std::string nom_interface_name = argv[1];

    if (argc == 2)
    {
        printf("No redundancy mode selected \n");
        socket_redundancy = std::make_shared<SocketNull>();
    }
    else
    {
        socket_redundancy = std::make_shared<Socket>();
        red_interface_name = argv[2];
    }

    selectInterface(nom_interface_name, red_interface_name);

    auto socket_nominal = std::make_shared<Socket>();
    try
    {
        socket_nominal->open(nom_interface_name);
        socket_redundancy->open(red_interface_name);
    }
    catch (std::exception const &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    auto report_redundancy = []()
    {
        printf("Redundancy has been activated due to loss of a cable \n");
    };

    std::shared_ptr<Link> link = std::make_shared<Link>(socket_nominal, socket_redundancy, report_redundancy);
    link->setTimeout(2ms);
    link->checkRedundancyNeeded();

    Bus bus(link);

    uint8_t initialSlaveCount = 0;
    std::vector<uint8_t> ingeniaIdList;

    uint8_t io_buffer[2048];
    try
    {
        bus.init();
        initialSlaveCount = static_cast<uint8_t>(bus.slaves().size());

        if (bus.slaves().size() == 0)
        {
            std::cout << "Error, no slave found on bus" << std::endl;
            return false;
        }
        printf("Found %d slaves on bus", initialSlaveCount);

        for (auto& slave: bus.slaves())
        {
            printInfo(slave);
            printESC(slave);
        }

        // Discover slave types
        for (uint8_t i = 0; i < initialSlaveCount; i++)
        {
            uint32_t data = 0;
            uint32_t size = sizeof(data);
            try
            {
                auto& slave = bus.slaves().at(i);
                bus.readSDO(slave, 0x1018, 2, Bus::Access::PARTIAL, (void*)&data, &size); // "Identity object" register from ingenia documentation
            }
            catch (ErrorCode const& e)
            {
                printErrorCode(e);
            }
            catch (std::exception const& e)
            {
                printException(e);
            }
            if (data == 61935618 || data == 61935617) // Ingenia product code - we can't use vendorID because we use the same...
            {
                ingeniaIdList.push_back(i);
            }
            else
            {
                std::cout << "Warning: unknown board type: " << data << std::endl;
            }
        }
        std::cout << "Found " << ingeniaIdList.size() << " ingenia drives on bus." << std::endl;  

        // Prepare mapping for ingenia
        const auto mapPDO = [&](const uint8_t slaveId, const uint16_t PDO_map, const uint32_t *data, const uint32_t dataSize, const uint32_t SM_map) -> void
        {
            uint8_t zeroU8 = 0;

/*
            // Unmap previous registers, setting 0 in PDO_MAP subindex 0
            bus.writeSDO(bus.slaves().at(slaveId), PDO_map, 0, false, const_cast<uint8_t *>(&zeroU8), sizeof(zeroU8));
            // Modify mapping, setting register address in PDO's subindexes from 0x1A00:01
            for (uint32_t i = 0; i < dataSize; i++)
            {
                uint8_t subIndex = static_cast<uint8_t>(i + 1);
                bus.writeSDO(bus.slaves().at(slaveId), PDO_map, subIndex, false, const_cast<uint32_t *>(&data[i]), sizeof(data[i]));
            }
            // Enable mapping by setting number of registers in PDO_MAP subindex 0
            uint8_t pdoMapSize = static_cast<uint8_t>(dataSize);
            bus.writeSDO(bus.slaves().at(slaveId), PDO_map, 0, false, const_cast<uint8_t *>(&pdoMapSize), sizeof(pdoMapSize));
*/

            uint8_t buffer[1024];
            std::memcpy(buffer + 2, data, dataSize * 4);
            buffer[0] = static_cast<uint8_t>(dataSize & 0xFF);
            buffer[1] = static_cast<uint8_t>((dataSize >> 8) & 0xFF);
            bus.writeSDO(bus.slaves().at(slaveId), PDO_map, 0, true, buffer, dataSize * 4 + 2);

            // Set PDO mapping to SM
            // Unmap previous mappings, setting 0 in SM_MAP subindex 0
            bus.writeSDO(bus.slaves().at(slaveId), SM_map, 0, false, const_cast<uint8_t *>(&zeroU8), sizeof(zeroU8));
            // Write first mapping (PDO_map) address in SM_MAP subindex 1
            bus.writeSDO(bus.slaves().at(slaveId), SM_map, 1, false, const_cast<uint16_t *>(&PDO_map), sizeof(PDO_map));
            // Save mapping count in SM (here only one PDO_MAP)
            uint8_t pdoMapSize = 1;
            bus.writeSDO(bus.slaves().at(slaveId), SM_map, 0, false, const_cast<uint8_t *>(&pdoMapSize), sizeof(pdoMapSize));
        };

        // Asign PDOs for each slave
        for (auto const& i: ingeniaIdList)
        {
            // Map TXPDO
            mapPDO(i, 0x1A00, pdo::tx_mapping, pdo::tx_mapping_count, 0x1C13);
            // Map RXPDO
            mapPDO(i, 0x1600, pdo::rx_mapping, pdo::rx_mapping_count, 0x1C12);
        }

        bus.createMapping(io_buffer);

        printf("Request SAFE OP\n");
        bus.requestState(State::SAFE_OP);
        bus.waitForState(State::SAFE_OP, 1s);
    }
    catch (ErrorCode const &e)
    {
        std::cerr << e.what() << ": " << ALStatus_to_string(e.code()) << std::endl;
        return 1;
    }
    catch (std::exception const &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    auto callback_error = [](DatagramState const & ds)
    {
        THROW_ERROR_DATAGRAM("something bad happened", ds);
    };
    auto false_alarm = [](DatagramState const &)
    { printf("previous error was a false alarm"); };

    try
    {
        bus.processDataRead(callback_error);
        bus.processDataWrite(false_alarm);
    }
    catch (...)
    {
    }

    try
    {
        bus.requestState(State::OPERATIONAL);
        bus.waitForState(State::OPERATIONAL, 100ms);
    }
    catch (ErrorCode const &e)
    {
        std::cerr << e.what() << ": " << ALStatus_to_string(e.code()) << std::endl;
        return 1;
    }
    catch (std::exception const &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    link->setTimeout(20ms);

    // Vector to store structs of each slave
    std::vector<CANOpenStateMachine> state_machines(bus.slaves().size());

    printf("Initializing communication with %ld slaves...\n", bus.slaves().size());

    // Slaves configuration
    std::vector<pdo::Output*> output_pdos(bus.slaves().size());
    std::vector<pdo::Input*> input_pdos(bus.slaves().size());

    for (size_t i = 0; i < bus.slaves().size(); ++i)
    {
        // Slave &ingenia = bus.slaves().at(0);
        Slave &slave = bus.slaves().at(i);
        printf("Mapping: Slave %ld -> Input Size: %d, Output Size: %d\n", i, slave.input.bsize, slave.output.bsize);

        // Asign input and output buffers
        output_pdos[i] = reinterpret_cast<pdo::Output *>(slave.output.data);
        input_pdos[i] = reinterpret_cast<pdo::Input *>(slave.input.data);

        // Configurate the state machine for each slave
        state_machines[i].setCommand(CANOpenCommand::ENABLE);

        // parameters cofiguration - Torque and position 
        output_pdos[i]->mode_of_operation = 5; // torque mode
        output_pdos[i]->target_torque = 0.03; //before: 3
        output_pdos[i]->max_current = 1.7; // before: 3990
        output_pdos[i]->target_position = input_pdos[i]->actual_position;
    }

    constexpr int64_t LOOP_NUMBER = 12 * 3600 * 1000; // 12 hours of execution 
    int64_t last_error = 0;
    
    for (int64_t i = 0; i < LOOP_NUMBER; ++i)
    {
        sleep(10ms);

        try
        {
            // Read and write data for all the slaves
            try
            {
            bus.sendLogicalRead(callback_error);
            bus.finalizeDatagrams();
            bus.processAwaitingFrames();
            }
            catch(...)
            {
                printf("logical access read\n");
            }

            try
            {
            bus.sendLogicalWrite(callback_error);
            bus.finalizeDatagrams();
            bus.processAwaitingFrames();
            }
            catch(...)
            {
                printf("logical access write\n");

            }

            bus.sendMailboxesReadChecks(callback_error);
            bus.sendReadMessages(callback_error);
            bus.finalizeDatagrams();
            bus.processAwaitingFrames();

        }
        catch (kickcat::ErrorDatagram const& e)
        {
            int64_t delta = i - last_error;
            last_error = i;
            std::cerr << e.what() << ": " << toString(e.state()) << " at " << i << " delta: " << delta << std::endl;
        }
        catch (std::exception const& e)
        {
            int64_t delta = i - last_error;
            last_error = i;
            std::cerr << e.what() << " at " << i << " delta: " << delta << std::endl;
        }

        // Process each slave individually 
        for (size_t j = 0; j < bus.slaves().size(); ++j)
        {
            state_machines[j].update(input_pdos[j]->status_word);
            output_pdos[j]->control_word = state_machines[j].getControlWord();

            // Check for emergencies in each slave
            Slave &slave = bus.slaves().at(j);
            if (slave.mailbox.emergencies.size() > 0)
            {
                for (auto &em : slave.mailbox.emergencies)
                {
                    std::cerr << "*~~~ Emergency received on Slave " << j << " @ " << i << " ~~~*" << std::endl;
                    std::cerr << registerToError(em.error_register);
                    std::cerr << codeToError(em.error_code);
                }
                slave.mailbox.emergencies.resize(0);
            }
        }
    }

    return 0;
}
