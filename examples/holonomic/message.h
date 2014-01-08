#ifndef _MESSAGE_H_
#define _MESSAGE_H_

//-----------------------------------------------------------------------------

#include "state.h"
#include "command.h"
#include "memory.h"

//-----------------------------------------------------------------------------
class message_c {
public:

  //---------------------------------------------------------------------------
  message_c( void ) { }

  //---------------------------------------------------------------------------
  virtual ~message_c( void ) {}

  //---------------------------------------------------------------------------
  virtual bool read( const message_c& from );
  virtual bool write( message_c& to );

};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
enum act_msg_type_e {
  MSG_UNDEFINED,
  MSG_COMMAND,
  MSG_NO_COMMAND,
  MSG_REQUEST,
  MSG_REPLY
};

//-----------------------------------------------------------------------------

struct act_msg_header_t {
  int pid;                                        // ProcessIDentifier
  int tid;                                        // ThreadIDentifier
  act_msg_type_e type;                            // Message Type
  unsigned long long ts;                          // TimeStamp
};

//-----------------------------------------------------------------------------

struct act_msg_body_t {
  pp_state_msg_t state;
  ship_command_msg_t command;
};

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
class act_msg_c : public message_c {
public:
  act_msg_header_t header;
  act_msg_body_t body;

  //---------------------------------------------------------------------------
  act_msg_c( void ) { 
    
  }

  //---------------------------------------------------------------------------
  virtual ~act_msg_c( void ) { 
    
  }

  //---------------------------------------------------------------------------
  virtual bool read( const act_msg_c& from ) {
    memcpy( &header, &from.header, sizeof(struct act_msg_header_t) );
    memcpy( &body.state, &from.body.state, sizeof(struct pp_state_msg_t) );
    memcpy( &body.command, &from.body.command, sizeof(struct ship_command_msg_t) );

    return true;
  }

  //---------------------------------------------------------------------------
  virtual bool write( act_msg_c& to ) {
    memcpy( &to.header, &header, sizeof(struct act_msg_header_t) );
    memcpy( &to.body.state, &body.state, sizeof(struct pp_state_msg_t) );
    memcpy( &to.body.command, &body.command, sizeof(struct ship_command_msg_t) );

    return true;
  }

  //---------------------------------------------------------------------------
  ship_state_msg_t predator( void ) {
    pp_state_c pp( body.state );
    return pp.pred_state().as_msg();
  }

  //---------------------------------------------------------------------------
  void predator( const ship_state_c& state ) {
    body.state.predator = state.as_msg();
  }

  //---------------------------------------------------------------------------
  ship_state_msg_t prey( void ) {
    pp_state_c pp( body.state );
    return pp.prey_state().as_msg();
  }

  //---------------------------------------------------------------------------
  void prey( const ship_state_c& state ) {
    body.state.prey = state.as_msg();
  }

  //---------------------------------------------------------------------------
  ship_state_msg_t state_msg( void ) {
    pp_state_c pp( body.state );
    return pp.prey_state().as_msg();
  }

  //---------------------------------------------------------------------------
  pp_state_c state( void ) {
    pp_state_c pp( body.state );
    return pp;
  }

  //---------------------------------------------------------------------------
  void state( const pp_state_c& q ) {
    body.state = q.as_msg();
  }

  //---------------------------------------------------------------------------
  ship_command_msg_t command_msg( void ) {
    ship_command_c u( body.command );
    return u.as_msg();
  }

  //---------------------------------------------------------------------------
  ship_command_c command( void ) {
    ship_command_c u( body.command );
    return u;
  }

  //---------------------------------------------------------------------------
  void command( const ship_command_c& u ) {
    body.command = u.as_msg();
  }

  //---------------------------------------------------------------------------
};


//------------------------------------------------------------------------------
class act_msg_buffer_c : public buffer_c< act_msg_c > {
public:

  //---------------------------------------------------------------------------
  act_msg_buffer_c( void ) { 

  }

  //---------------------------------------------------------------------------
  virtual ~act_msg_buffer_c( void ) { 
    
  }

  //---------------------------------------------------------------------------
  virtual bool open( void ) {
    return true;
  }

  //---------------------------------------------------------------------------
  virtual void close( void ) {

  }

  //---------------------------------------------------------------------------
  virtual bool read( const act_msg_c& from ) {
    memcpy( &buffer->header, &from.header, sizeof(struct act_msg_header_t) );
    memcpy( &buffer->body.state, &from.body.state, sizeof(struct pp_state_msg_t) );
    memcpy( &buffer->body.command, &from.body.command, sizeof(struct ship_command_msg_t) );

    return true;
  }

  //---------------------------------------------------------------------------
  virtual bool write( act_msg_c& to ) {
    memcpy( &to.header, &buffer->header, sizeof(struct act_msg_header_t) );
    memcpy( &to.body.state, &buffer->body.state, sizeof(struct pp_state_msg_t));
    memcpy( &to.body.command, &buffer->body.command, sizeof(struct ship_command_msg_t) );

    return true;
  }

  //---------------------------------------------------------------------------
};

//------------------------------------------------------------------------------
//*/
#endif // _MESSAGE_H_
