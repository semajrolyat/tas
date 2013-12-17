#include <space.h>

#include <maze.h>

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
space_c::space_c( void ) { 

}

//-----------------------------------------------------------------------------
space_c::space_c( gazebo::physics::WorldPtr _world ) { 
  world = _world;

  //sdf::SDF obssdf = make_maze();

  //world->InsertModelSDF( obssdf );
/*
  static bool first_call = true;
  if( first_call ) {
    maze_c maze( world );
    maze.create( 1 );

    gazebo::physics::ModelPtr mazemodel = world->GetModel("maze");
    if( !mazemodel ) 
      gzerr << "Unable to find model: maze\n";
    first_call = false;

    gazebo::physics::Model_V modelvector = world->GetModels();
    for( unsigned i = 0; i < modelvector.size(); i++ ) {
      std::string name = modelvector[i]->GetName();
      std::cout << name << std::endl;
    }
    world->PrintEntityTree();
  }
*/
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
space_c::~space_c( void ) {

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
bool space_c::read( void ) {

  // get the reference to the spatial bound
  gazebo::physics::ModelPtr space = world->GetModel("pen");
  if( !space ) {
    gzerr << "Unable to find model: pen\n";
    return false;
  }
  // compute & cache the spatial bound aabb

  // Space is empty inside the boundary, but the boundary is made up of 
  // thin walls not just a cube.
  gazebo::physics::Link_V space_links = space->GetLinks();
  if( space_links.size() != 6 ) {
    // very basic validation.  can still be malformed even if has correct 
    // number of planes
    gzerr << "Spatial boundary is malformed: model=pen\n";
    return false;
  }

  gazebo::math::Vector3 c( 0.0, 0.0, 0.0 );
  gazebo::math::Vector3 e( 0.0, 0.0, 0.0 );
  for( unsigned i = 0; i < space_links.size(); i++ ) {
    gazebo::physics::LinkPtr link = space_links[i];
    gazebo::math::Box gzbb = link->GetBoundingBox();
    c += gzbb.GetCenter();
    // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
    e = gazebo::math::Vector3( std::max(gzbb.GetCenter().x, e.x), std::max(gzbb.GetCenter().y, e.y),std::max(gzbb.GetCenter().z, e.z) );
  }
  bounds = aabb_c( c, e );

  planes.resize(6);
  // east
  planes[0]=plane_c(Ravelin::Vector3d(c.x+e.x,0,0),Ravelin::Vector3d(-1,0,0));
  // west
  planes[1]=plane_c(Ravelin::Vector3d(c.x-e.x,0,0),Ravelin::Vector3d(1,0,0));
  // north
  planes[2]=plane_c(Ravelin::Vector3d(0,c.y+e.y,0),Ravelin::Vector3d(0,-1,0));
  // south
  planes[3]=plane_c(Ravelin::Vector3d(0,c.y-e.y,0),Ravelin::Vector3d(0,1,0));
  // up
  planes[4]=plane_c(Ravelin::Vector3d(0,0,c.z+e.z),Ravelin::Vector3d(0,0,-1));
  // down
  planes[5]=plane_c(Ravelin::Vector3d(0,0,c.z-e.z),Ravelin::Vector3d(0,0,1));

  return true;
}

//-----------------------------------------------------------------------------
sdf::SDF space_c::make_maze( void ) {

  const int SEED = 1;
  srand( SEED );

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
 
  obssdf.Write("maze.sdf");

  return obssdf;

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

//-----------------------------------------------------------------------------
int main( void ) {
  space_c::make_maze();
}

//-----------------------------------------------------------------------------

