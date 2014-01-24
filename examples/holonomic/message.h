#ifndef _MESSAGE_H_
#define _MESSAGE_H_

//-----------------------------------------------------------------------------

#include "time.h"
#include "state.h"
#include "command.h"

//-----------------------------------------------------------------------------
enum act_msg_requestor_e {
  REQUESTOR_NONE,
  REQUESTOR_PLANNER,
  REQUESTOR_PREDATOR,
  REQUESTOR_PREY
};

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
  int pid;                                      // ProcessIDentifier
  int tid;                                      // ThreadIDentifier
  act_msg_type_e type;                          // Message Type
  timestamp_t ts;                               // TimeStamp
  simtime_t time;
  act_msg_requestor_e requestor;
};

//-----------------------------------------------------------------------------

struct act_msg_body_t {
  pp_state_msg_t state;
  ship_command_msg_t command;
};

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
class act_msg_c {
public:
  act_msg_header_t header;
  act_msg_body_t body;

  //---------------------------------------------------------------------------
  act_msg_c( void ) { 
    header.pid = 0;
    header.tid = 0;
    header.ts.cycle = 0;
    header.time.seconds = 0;
    header.type = MSG_UNDEFINED;
    header.requestor = REQUESTOR_NONE;

    body.state.predator.x = 0;
    body.state.predator.y = 0;
    body.state.predator.z = 0;
    body.state.predator.qx = 0;
    body.state.predator.qy = 0;
    body.state.predator.qz = 0;
    body.state.predator.qw = 1;
    body.state.predator.dx = 0;
    body.state.predator.dy = 0;
    body.state.predator.dz = 0;
    body.state.predator.drotx = 0;
    body.state.predator.droty = 0;
    body.state.predator.drotz = 0;

    body.state.prey.x = 0;
    body.state.prey.y = 0;
    body.state.prey.z = 0;
    body.state.prey.qx = 0;
    body.state.prey.qy = 0;
    body.state.prey.qz = 0;
    body.state.prey.qw = 1;
    body.state.prey.dx = 0;
    body.state.prey.dy = 0;
    body.state.prey.dz = 0;
    body.state.prey.drotx = 0;
    body.state.prey.droty = 0;
    body.state.prey.drotz = 0;

    body.command.fx = 0;
    body.command.fy = 0;
    body.command.fz = 0;
    body.command.tx = 0;
    body.command.ty = 0;
    body.command.tz = 0;
  }

  //---------------------------------------------------------------------------
  act_msg_c( const act_msg_c& msg ) {
    header.pid = msg.header.pid;
    header.tid = msg.header.tid;
    header.ts.cycle = msg.header.ts.cycle;
    header.time.seconds = msg.header.time.seconds;
    header.type = msg.header.type;
    header.requestor = msg.header.requestor;

    body.state.predator.x = msg.body.state.predator.x;
    body.state.predator.y = msg.body.state.predator.y;
    body.state.predator.z = msg.body.state.predator.z;
    body.state.predator.qx = msg.body.state.predator.qx;
    body.state.predator.qy = msg.body.state.predator.qy;
    body.state.predator.qz = msg.body.state.predator.qz;
    body.state.predator.qw = msg.body.state.predator.qw;
    body.state.predator.dx = msg.body.state.predator.dx;
    body.state.predator.dy = msg.body.state.predator.dy;
    body.state.predator.dz = msg.body.state.predator.dz;
    body.state.predator.drotx = msg.body.state.predator.drotx;
    body.state.predator.droty = msg.body.state.predator.droty;
    body.state.predator.drotz = msg.body.state.predator.drotz;

    body.state.prey.x = msg.body.state.prey.x;
    body.state.prey.y = msg.body.state.prey.y;
    body.state.prey.z = msg.body.state.prey.z;
    body.state.prey.qx = msg.body.state.prey.qx;
    body.state.prey.qy = msg.body.state.prey.qy;
    body.state.prey.qz = msg.body.state.prey.qz;
    body.state.prey.qw = msg.body.state.prey.qw;
    body.state.prey.dx = msg.body.state.prey.dx;
    body.state.prey.dy = msg.body.state.prey.dy;
    body.state.prey.dz = msg.body.state.prey.dz;
    body.state.prey.drotx = msg.body.state.prey.drotx;
    body.state.prey.droty = msg.body.state.prey.droty;
    body.state.prey.drotz = msg.body.state.prey.drotz;

    body.command.fx = msg.body.command.fx;
    body.command.fy = msg.body.command.fy;
    body.command.fz = msg.body.command.fz;
    body.command.tx = msg.body.command.tx;
    body.command.ty = msg.body.command.ty;
    body.command.tz = msg.body.command.tz;
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
  virtual bool write( act_msg_c& to ) const {
    memcpy( &to.header, &header, sizeof(struct act_msg_header_t) );
    memcpy( &to.body.state, &body.state, sizeof(struct pp_state_msg_t) );
    memcpy( &to.body.command, &body.command, sizeof(struct ship_command_msg_t) );

    return true;
  }
  //---------------------------------------------------------------------------
  // Abstract implementation
  virtual size_t size( void ) const {
    return sizeof( act_msg_c );
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
  pp_state_c state( void ) const {
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
  ship_command_c command( void ) const {
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

#endif // _MESSAGE_H_
