/// This file is auto generated by od_generator.

#include "kickcat/CoE/OD.h"

namespace kickcat::CoE
{
    CoE::Dictionary createOD()
    {
        CoE::Dictionary dictionary;

        {
            static CoE::Object object
            {
                0x1018,
                CoE::ObjectCode::RECORD,
                "Identity Object",
                {}
            };
            CoE::addEntry(object,0,8,7,static_cast<CoE::DataType>(5),"Subindex 000",0x4);
            CoE::addEntry(object,1,32,7,static_cast<CoE::DataType>(7),"Vendor ID",0x29c);
            CoE::addEntry(object,2,32,7,static_cast<CoE::DataType>(7),"Product code",0x32);
            CoE::addEntry(object,3,32,7,static_cast<CoE::DataType>(7),"Revision number",0x0);
            CoE::addEntry(object,4,32,7,static_cast<CoE::DataType>(7),"Serial number",0xcafedeca);
            dictionary.push_back(std::move(object));
        }

        {
            static CoE::Object object
            {
                0x1600,
                CoE::ObjectCode::RECORD,
                "RxPDO Map 1",
                {}
            };
            CoE::addEntry(object,0,8,15,static_cast<CoE::DataType>(5),"Subindex 000",0x1);
            CoE::addEntry(object,1,32,15,static_cast<CoE::DataType>(7),"RxPDO Map 1 Element 1",0x10);
            dictionary.push_back(std::move(object));
        }

        {
            static CoE::Object object
            {
                0x1a00,
                CoE::ObjectCode::RECORD,
                "TxPDO Map 1",
                {}
            };
            CoE::addEntry(object,0,8,15,static_cast<CoE::DataType>(5),"Subindex 000",0x1);
            CoE::addEntry(object,1,32,15,static_cast<CoE::DataType>(7),"TxPDO Map 1 Element 1",0xe0);
            dictionary.push_back(std::move(object));
        }

        {
            static CoE::Object object
            {
                0x1c00,
                CoE::ObjectCode::ARRAY,
                "Sync manager type",
                {}
            };
            CoE::addEntry(object,0,8,7,static_cast<CoE::DataType>(5),"Subindex 0",0x4);
            CoE::addEntry(object,1,8,7,static_cast<CoE::DataType>(5),"Subindex 1",0x1);
            CoE::addEntry(object,2,8,7,static_cast<CoE::DataType>(5),"Subindex 2",0x2);
            CoE::addEntry(object,3,8,7,static_cast<CoE::DataType>(5),"Subindex 3",0x3);
            CoE::addEntry(object,4,8,7,static_cast<CoE::DataType>(5),"Subindex 4",0x4);
            dictionary.push_back(std::move(object));
        }

        {
            static CoE::Object object
            {
                0x1c12,
                CoE::ObjectCode::RECORD,
                "RxPDO assign",
                {}
            };
            CoE::addEntry(object,0,8,15,static_cast<CoE::DataType>(5),"Subindex 000",0x1);
            CoE::addEntry(object,1,32,15,static_cast<CoE::DataType>(7),"RxPDO assign Element 1",0x1600);
            dictionary.push_back(std::move(object));
        }

        {
            static CoE::Object object
            {
                0x1c13,
                CoE::ObjectCode::RECORD,
                "TxPDO assign",
                {}
            };
            CoE::addEntry(object,0,8,15,static_cast<CoE::DataType>(5),"Subindex 000",0x1);
            CoE::addEntry(object,1,32,15,static_cast<CoE::DataType>(7),"TxPDO assign Element 1",0x1a00);
            dictionary.push_back(std::move(object));
        }

         return dictionary;
    }
}
