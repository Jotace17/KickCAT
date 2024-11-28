#ifndef SLAVE_STACK_INCLUDE_SLAVE_H_
#define SLAVE_STACK_INCLUDE_SLAVE_H_


#include "PDO.h"
#include "kickcat/AbstractESC.h"
#include "kickcat/Mailbox.h"
#include "kickcat/ESMStates.h"


// TODO: to rename file => namespace master and slave
namespace kickcat
{
    class Slave final
    {
    public:
        Slave(AbstractESC* esc, PDO* pdo);

        void set_mailbox(mailbox::response::Mailbox* mbx);
        void start();
        void routine();
        State getState();
        void setOutputDataValid(bool isValid);

    private:
        AbstractESC* esc_;
        mailbox::response::Mailbox* mbx_;
        PDO* pdo_;

        ESM::Init init_{*esc_, *pdo_};
        ESM::PreOP preOp_{*esc_, *pdo_};
        ESM::SafeOP safeOP_{*esc_, *pdo_};
        ESM::OP OP_{*esc_, *pdo_};
        ESM::StateMachine stateMachine_{*esc_, {{&init_, &preOp_, &safeOP_, &OP_}}};
    };
}

#endif
