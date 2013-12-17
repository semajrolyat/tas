#ifndef _MAZE_H_
#define _MAZE_H_

//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <valarray>
#include <limits>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <boost/shared_ptr.hpp>

//-----------------------------------------------------------------------------

class maze_c {
public:
  // the gazebo reference to the world in which the maze is located
  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr model;

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  maze_c( void ) {

  }

  maze_c( gazebo::physics::WorldPtr _world ) : world(_world) 
  {

  }
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~maze_c( void ) {

  }

  //---------------------------------------------------------------------------
  //bool read( void );

  //---------------------------------------------------------------------------
  void create( const unsigned& seed ) {
    srand( seed );

    sdf::SDF obssdf;
    std::stringstream ssname, sspose, sstemp, ssdims;

    const double EXTENS_X = 25.0;
    const double EXTENS_Y = 25.0;
    const double EXTENS_Z = 25.0;
 
    const double MAX_DIM = 5.0;

    const int OBSTACLES = 100;

    boost::shared_ptr <sdf::Element> obstacle( new sdf::Element() );
    obssdf.root->InsertElement( obstacle );
    obstacle->SetParent( obssdf.root );
    obstacle->SetName( "model" );
    obstacle->AddAttribute("name", "string", "maze", true, "" );

    boost::shared_ptr <sdf::Element> obs_static( new sdf::Element() );
    obstacle->InsertElement( obs_static );
    obs_static->SetParent( obstacle );
    obs_static->SetName("static");
    obs_static->AddValue( "string", "true", true, "" );  //boolean

    for( unsigned i = 0; i < OBSTACLES; i++ ) {

      ssname.str("");
      ssname.clear();
      ssname << "obstacle_" << i;

      double x, y, z;
      x = ((2.0 * (double) rand() / RAND_MAX) - 1.0) * EXTENS_X;
      y = ((2.0 * (double) rand() / RAND_MAX) - 1.0) * EXTENS_Y;
      z = ((2.0 * (double) rand() / RAND_MAX) - 1.0) * EXTENS_Z;

      sspose.str("");
      sspose.clear();
      sspose << x << " " << y << " " << z << " 0 0 0";

      double l, w, h;
      l = (double)rand() / RAND_MAX * MAX_DIM;
      w = (double)rand() / RAND_MAX * MAX_DIM;
      h = (double)rand() / RAND_MAX * MAX_DIM;

      ssdims.str("");
      ssdims.clear();
      ssdims << l << " " << w << " " << h;

      sstemp.str("");
      sstemp.clear();
      sstemp << ssname.str() << "_link";
      boost::shared_ptr <sdf::Element> link( new sdf::Element() );
      obstacle->InsertElement( link );
      link->SetParent( obstacle );
      link->SetName( "link" );
      link->AddAttribute("name", "string", sstemp.str(), true, "" );

      boost::shared_ptr <sdf::Element> link_pose( new sdf::Element() );
      link->InsertElement( link_pose );
      link_pose->SetParent( link );
      link_pose->SetName("pose");
      link_pose->AddValue( "string", sspose.str(), true, "" );

      sstemp.str("");
      sstemp.clear();
      sstemp << ssname.str() << "_col";
      boost::shared_ptr <sdf::Element> col( new sdf::Element() );
      link->InsertElement( col );
      col->SetParent( link );
      col->SetName( "collision" );
      col->AddAttribute( "name", "string", sstemp.str(), true, "" );
 
      boost::shared_ptr <sdf::Element> col_pose( new sdf::Element() );
      col->InsertElement( col_pose );
      col_pose->SetParent( col );
      col_pose->SetName("pose");
      col_pose->AddValue( "string", "0 0 0 0 0 0", true, "" );

      boost::shared_ptr <sdf::Element> col_geometry( new sdf::Element() );
      col->InsertElement( col_geometry );
      col_geometry->SetParent( col );
      col_geometry->SetName( "geometry" );
  
      boost::shared_ptr <sdf::Element> col_box( new sdf::Element() );
      col_geometry->InsertElement( col_box );
      col_box->SetParent( col_geometry );
      col_box->SetName( "box" );

      boost::shared_ptr <sdf::Element> col_box_size( new sdf::Element() );
      col_box->InsertElement( col_box_size );
      col_box_size->SetParent( col_box );
      col_box_size->SetName( "size" );
      col_box_size->AddValue( "string", ssdims.str(), true, "" );
  
      sstemp.str("");
      sstemp.clear();
      sstemp << ssname.str() << "_vis";
      boost::shared_ptr <sdf::Element> vis( new sdf::Element() );
      link->InsertElement( vis );
      vis->SetParent( link );
      vis->SetName( "visual" );
      vis->AddAttribute( "name", "string", sstemp.str(), true, "" );
 
      boost::shared_ptr <sdf::Element> vis_pose( new sdf::Element() );
      vis->InsertElement( vis_pose );
      vis_pose->SetParent( vis );
      vis_pose->SetName("pose");
      vis_pose->AddValue( "string", "0 0 0 0 0 0", true, "" );

      boost::shared_ptr <sdf::Element> vis_geometry( new sdf::Element() );
      vis->InsertElement( vis_geometry );
      vis_geometry->SetParent( vis );
      vis_geometry->SetName("geometry");
  
      boost::shared_ptr <sdf::Element> vis_box( new sdf::Element() );
      vis_geometry->InsertElement( vis_box );
      vis_box->SetParent( vis_geometry );
      vis_box->SetName("box");

      boost::shared_ptr <sdf::Element> vis_box_size( new sdf::Element() );
      vis_box->InsertElement( vis_box_size );
      vis_box_size->SetParent( vis_box );
      vis_box_size->SetName("size");
      vis_box_size->AddValue( "string", ssdims.str(), true, "" );
  
      boost::shared_ptr <sdf::Element> vis_material( new sdf::Element() );
      vis->InsertElement( vis_material );
      vis_material->SetParent( vis );
      vis_material->SetName("material");
  
      boost::shared_ptr <sdf::Element> vis_mat_script( new sdf::Element() );
      vis_material->InsertElement( vis_mat_script );
      vis_mat_script->SetParent( vis_material );
      vis_mat_script->SetName("script");

      boost::shared_ptr <sdf::Element> vis_mat_uri( new sdf::Element() );
      vis_mat_script->InsertElement( vis_mat_uri );
      vis_mat_uri->SetParent( vis_mat_script );
      vis_mat_uri->SetName("uri");
      vis_mat_uri->AddValue( "string", "file://media/materials/scripts/gazebo.material", true, "" );

      boost::shared_ptr <sdf::Element> vis_mat_name( new sdf::Element() );
      vis_mat_script->InsertElement( vis_mat_name );
      vis_mat_name->SetParent( vis_mat_script );
      vis_mat_name->SetName("name");
      vis_mat_name->AddValue( "string", "Gazebo/Yellow", true, "" );

      //std::string s = obstacle->ToString( std::string("") );
      //std::cout <<i s;
    }
 
    //obssdf.Write("maze.sdf");

    //boost::shared_ptr<gazebo::physics::Model> model( new gazebo::physics::Model( NULL ) );

    //model->Load( obstacle );
    //model->SetWorld( world );
    //model->GetSDF()->SetParent()
    //world->InsertModelString( model->GetSDF()->ToString("") );
    //world->InsertModelString( obstacle->ToString("") );

    std::string strmaze = "<?xml version=\"1.0\" ?>\n<sdf version=\"1.4\">\n";
    strmaze += obstacle->ToString("");
    strmaze += "</sdf>\n";
    world->InsertModelString( strmaze );
    world->UpdateStateSDF();
 

  /*
    // Add the appropriate headers
    std::ifstream infile( "maze.sdf" );
    std::string str;
  
    int line = 0;
    int lines = 0;
 
    if( infile.is_open() ) {
      while( getline( infile, str ) ) {
        lines++;
      }
      infile.close();
    }

    std::ofstream outfile( "models/maze/model.sdf" );
    std::ifstream infile2( "maze.sdf" );
    if( infile2.is_open() ) {
      while( getline( infile2, str ) ) {
        line++;
        if( line == 1 ) {
          outfile << "<?xml version=\"1.0\" ?>\n<sdf version=\"1.4\">\n";
        } else if( line == lines ) {
          outfile << "</sdf>\n";
        } else {
          outfile << str << std::endl;
        }
      }
      outfile.close();
      infile.close();
    }
  */
  }
};

//-----------------------------------------------------------------------------

#endif // _MAZE_H_


