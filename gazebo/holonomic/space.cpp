#include <space.h>

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
GZ_REGISTER_MODEL_PLUGIN( space_c )

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
space_c::space_c( void ) { 

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
space_c::~space_c( void ) {
    //gazebo::event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void space_c::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model = _model;
  world = _model->GetWorld();

  const int SEED = 1;
  srand( SEED );

  sdf::SDF obssdf;
  std::stringstream ssname, sspose, sstemp, ssdims;

  const double EXTENS_X = 25.0;
  const double EXTENS_Y = 25.0;
  const double EXTENS_Z = 25.0;
 
  const double MAX_DIM = 5.0;

  const int OBSTACLES = 50;

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
    x = ((2.0 * (double)rand() / (double)RAND_MAX) - 1.0) * EXTENS_X;
    y = ((2.0 * (double)rand() / (double)RAND_MAX) - 1.0) * EXTENS_Y;
    z = ((2.0 * (double)rand() / (double)RAND_MAX) - 1.0) * EXTENS_Z;

    sspose.str("");
    sspose.clear();
    sspose << x << " " << y << " " << z << " 0 0 0";

    double l, w, h;
    l = ((double)rand() / (double)RAND_MAX) * MAX_DIM;
    w = ((double)rand() / (double)RAND_MAX) * MAX_DIM;
    h = ((double)rand() / (double)RAND_MAX) * MAX_DIM;

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
  obssdf.Write("test.sdf");
  world->InsertModelSDF( obssdf );


}
/*
//-----------------------------------------------------------------------------
void ship_c::Update( ) {

}
*/
//-----------------------------------------------------------------------------
/*
void ship_c::Reset( ) {

}
*/

//-----------------------------------------------------------------------------

