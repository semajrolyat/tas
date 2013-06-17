#ifndef _OPTIONS_H_
#define _OPTIONS_H_

#include <string>
#include <assert.h>
#include <stdlib.h>

namespace Moby {

class Options {
public:
    Options( void );
    virtual ~Options( void );

    bool check_osg()
    {
      #ifdef USE_OSG
      return true;
      #else
      return false;
      #endif
    }

    std::string usage () {
        std::string result = "";

        return result;
    }

    void process( int argc, char** argv ) {

        /*
        // check that syntax is ok
        if (argc < 2) {
            std::cerr << "syntax: driver [OPTIONS] <xml file>" << std::endl;
            std::cerr << "        (see README for OPTIONS)" << std::endl;
            return -1;
        }
        */

        // get all options
        for (int i=1; i< argc-1; i++) {
            // get the option
            std::string option(argv[i]);

            // process options
            if ( option == "-r") {
                // render
                ONSCREEN_RENDER = true;
                UPDATE_GRAPHICS = true;
                check_osg();
            } else if ( option.find("-of") != std::string::npos ) {
                // output frame rate
                OUTPUT_FRAME_RATE = true;
            } else if ( option.find("-ot") != std::string::npos ) {
                // output timing
                OUTPUT_TIMINGS = true;
            } else if ( option.find("-oi") != std::string::npos ) {
                // output iteration number
                OUTPUT_ITER_NUM = true;
            } else if ( option.find("-or") != std::string::npos ) {
                // output simulation rate
                OUTPUT_SIM_RATE = true;
            } else if ( option.find("-v=") != std::string::npos ) {
                // ?
                UPDATE_GRAPHICS = true;
                check_osg();
                THREED_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
                assert( THREED_IVAL >= 0);
            } else if ( option.find("-i=") != std::string::npos ) {
                // ?
                check_osg();
                UPDATE_GRAPHICS = true;
                IMAGE_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
                assert(IMAGE_IVAL >= 0);
            } else if ( option.find("-t") != std::string::npos ) {
                // output
                OUTPUT_TO_TIME = true;
            } else if ( option.find("-s=") != std::string::npos ) {
                // set step size
                STEP_SIZE = std::atof(&argv[i][ONECHAR_ARG]);
                assert(STEP_SIZE >= 0.0 && STEP_SIZE < 1);
            } else if ( option.find("-lf=") != std::string::npos ) {
                // set log file
                std::string fname(&argv[i][TWOCHAR_ARG]);
                OutputToFile::stream.open(fname.c_str());                       // refactor
                // Don't open the file here!

                // instead redirect.
                LOG_FILE_PATH = std::string(&argv[i][TWOCHAR_ARG]);
            } else if ( option.find("-l=") != std::string::npos ) {
                // set logging level
                LOG_REPORTING_LEVEL = std::atoi(&argv[i][ONECHAR_ARG]);


                Log<OutputToFile>::reporting_level = LOG_REPORTING_LEVEL;       // refactor
                // Don't write to the file here!
            } else if ( option.find("-lt=") != std::string::npos ) {
                // set time to start logging
                LOG_START = std::atoi(&argv[i][TWOCHAR_ARG]);
            } else if ( option.find("-lp=") != std::string::npos ) {
                // set time to stop logging
                LOG_STOP = std::atoi(&argv[i][TWOCHAR_ARG]);
            } else if ( option.find("-mi=") != std::string::npos ) {
                // set max iteration
                MAX_ITER = std::atoi(&argv[i][TWOCHAR_ARG]);
                assert(MAX_ITER > 0);
            } else if ( option.find("-mt=") != std::string::npos ) {
                // set max time
                MAX_TIME = std::atof(&argv[i][TWOCHAR_ARG]);
                assert(MAX_TIME > 0);
            } else if ( option.find("-x=") != std::string::npos ) {
                // ?
                check_osg();
                //scene_path = std::string(&argv[i][ONECHAR_ARG]);                // refactor
                SCENE_PATH = std::string(&argv[i][ONECHAR_ARG]);
            } else if ( option.find("-p=") != std::string::npos ) {
                // set plugin
                read_plugin(&argv[i][ONECHAR_ARG]);                             // refactor

                // instead generate a list of plugin paths to be read in a separate step
            } else if ( option.find("-y=") != std::string::npos ) {
                // ?
                strcpy(THREED_EXT, &argv[i][ONECHAR_ARG]);
            } else if ( option.find("-vcp") != std::string::npos ) {
                // visualize contact points
                RENDER_CONTACT_POINTS = true;
            }
        }
    }

    /// Horizontal and vertical resolutions for offscreen-rendering
    const unsigned HORZ_RES = 1024;
    const unsigned VERT_RES = 768;

    /// Beginning iteration for logging
    unsigned LOG_START = 0;

    /// Ending iteration for logging
    unsigned LOG_STOP = std::numeric_limits<unsigned>::max();

    /// The logging reporting level
    unsigned LOG_REPORTING_LEVEL = 0;

    /*
    /// Used for timing
    //clock_t start_time;
    */
    /// The default simulation step size
    const Real DEFAULT_STEP_SIZE = .001;

    /// The simulation step size
    Real STEP_SIZE = DEFAULT_STEP_SIZE;
    /*
    /// The time of the first simulation step
    Real FIRST_STEP_TIME = -1;
    */
    /*
    /// The time of the last simulation step
    Real LAST_STEP_TIME = 0;
    */
    /*
    /// The current simulation iteration
    unsigned ITER = 0;
    */
    /// Interval for offscreen renders (0=offscreen renders disabled)
    unsigned IMAGE_IVAL = 0;

    /// Interval for 3D outputs (0=3D outputs disabled)
    unsigned THREED_IVAL = 0;

    /// Determines whether to do onscreen rendering (false by default)
    bool ONSCREEN_RENDER = false;

    /// Determines whether to output timings
    bool OUTPUT_TIMINGS = false;

    /// Extension/format for 3D outputs (default=Wavefront obj)
    char THREED_EXT[5] = "obj";

    /// Determines whether to update graphics (false by default, but certain
    /// options will set to true)
    bool UPDATE_GRAPHICS = false;

    /// If true, outputs (image, vrml) tied to simulation time rather than actual iteration count
    bool OUTPUT_TO_TIME = false;

    /// The maximum number of iterations (default infinity)
    unsigned MAX_ITER = std::numeric_limits<unsigned>::max();

    /// The maximum time of the simulation (default infinity)
    Real MAX_TIME = std::numeric_limits<Real>::max();

    /*
    /// The total (CPU) clock time used by the simulation
    Real TOTAL_TIME = 0.0;
    */
    /*
    /// Last 3D output iteration and time output
    unsigned LAST_3D_WRITTEN = -1;
    Real LAST_3D_WRITTEN_T = -std::numeric_limits<Real>::max()/2.0;
    */
    /*
    /// Last image iteration output
    unsigned LAST_IMG_WRITTEN = -1;
    Real LAST_IMG_WRITTEN_T = -std::numeric_limits<Real>::max()/2.0;
    */
    /// Outputs to stdout
    bool OUTPUT_FRAME_RATE = false;
    bool OUTPUT_ITER_NUM = false;
    bool OUTPUT_SIM_RATE = false;

    /// Render Contact Points
    bool RENDER_CONTACT_POINTS = false;

    //-- new fields --
    std::string LOG_FILE_PATH;
    std::string SCENE_PATH;

};

}
#endif // _OPTIONS_H_
