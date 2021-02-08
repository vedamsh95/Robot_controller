#include "lin.h"
#include "Vector.h"
#include "iostream"
#include "sdir_ctrl.h"
#include <array>
#include <ConfigProvider.h>

Trajectory* Lin::get_lin_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg, double speed, double acceleration)
{
    //TODO: IMPLEMENT! implement the computation of a lin trajectory with the corresponding velocity profile
    Trajectory* trajectory = new Trajectory();
    // Step1: Calculate start position and end position from the configs with direct kinematics
    FwKinematics fwKinematics;
    SixDPos* start_pos = fwKinematics.get_fw_kinematics(_start_cfg);
    SixDPos* end_pos = fwKinematics.get_fw_kinematics(_end_cfg);
    //convert start_cfg to radian

    double joint1_startpos = _start_cfg->operator[](0);
    Configuration* new_start_cfg = new Configuration({_start_cfg->operator[](0) * M_PII / 180, _start_cfg->operator[](1) * M_PII / 180,
                                                      _start_cfg->operator[](2) * M_PII / 180, _start_cfg->operator[](3) * M_PII / 180,
                                                      _start_cfg->operator[](4) * M_PII / 180, _start_cfg->operator[](5) * M_PII / 180});

    double timesteps = ConfigProvider::getInstance().getsteps_per_second();

    //write X,Y,Z into Vector
    Vector<double, 3> start_pos_vec, start_ori_vec;
    start_pos_vec[0] = start_pos->get_X();
    start_pos_vec[1] = start_pos->get_Y();
    start_pos_vec[2] = start_pos->get_Z();
    start_ori_vec[0] = start_pos->get_A();
    start_ori_vec[1] = start_pos->get_B();
    start_ori_vec[2] = start_pos->get_C();

    Vector<double, 3> end_pos_vec, end_ori_vec;
    end_pos_vec[0] = end_pos->get_X();
    end_pos_vec[1] = end_pos->get_Y();
    end_pos_vec[2] = end_pos->get_Z();
    end_ori_vec[0] = end_pos->get_A();
    end_ori_vec[1] = end_pos->get_B();
    end_ori_vec[2] = end_pos->get_C();

    //------------------add this from robot class-----------------------------------------------------------------------
    //setting max_velocity values for robot joints in °/s
    Vector<double, 6> joint_max_vel;
    joint_max_vel[0] = ConfigProvider::getInstance().getJoint1_max_vel();
    joint_max_vel[1] = ConfigProvider::getInstance().getJoint2_max_vel();
    joint_max_vel[2] = ConfigProvider::getInstance().getJoint3_max_vel();
    joint_max_vel[3] = ConfigProvider::getInstance().getJoint4_max_vel();
    joint_max_vel[4] = ConfigProvider::getInstance().getJoint5_max_vel();
    joint_max_vel[5] = ConfigProvider::getInstance().getJoint6_max_vel();

    //-------------------------set input speed and acceleration to new variables and check limits-----------------------
    double a_max_limit = ConfigProvider::getInstance().getMax_accel();
    double v_max_limit = 3;
    double a_max, v_max;
    if(acceleration > 0 && acceleration <= a_max_limit){
        a_max = acceleration;
    }else{
        a_max = a_max_limit;
    }

    if(speed > 0 && speed <= v_max_limit){
        v_max = speed;
    }else{
        v_max = v_max_limit;
    }


    //------calculate time when we reach max velocity in s--------------------------------------------------------------
    double t_c = v_max/a_max;
    //-------------------calculate distance of travel of the Endeffector in m-------------------------------------------

    double total_distance = sqrt( pow( end_pos->get_X()-start_pos->get_X(), 2 )
            + pow( end_pos->get_Y()-start_pos->get_Y(), 2 ) + pow( end_pos->get_Z()-start_pos->get_Z(), 2 ) );
    //calculate distances for orientation (probably not correct)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    double a_diff = (end_ori_vec[0] - start_ori_vec[0]);
    double b_diff = (end_ori_vec[1] - start_ori_vec[1]);
    double c_diff = (end_ori_vec[2] - start_ori_vec[2]);

    //calculate total time (when we reach end point for trapezoidal profile)
    double t_f = t_c + (total_distance / v_max);
    std::cout << "Lin: ------------------------------------------------------" << std::endl;
    std::cout << "Start Position (X,Y,Z)" << std::endl;
    start_pos_vec.output();
    std::cout << "End Position (X,Y,Z)" << std::endl;
    end_pos_vec.output();
    std::cout << "Start Orientation (A,B,C)" << std::endl;
    start_ori_vec.output();
    std::cout << "End Orientation (A,B,C)" << std::endl;
    end_ori_vec.output();
    std::cout << "maximum acceleration of Endeffector = " << a_max << std::endl;
    std::cout << "maximum velocity of Endeffector = " << v_max << std::endl;
    std::cout << "time when we reach max_velocity (t_c) = " << t_c << std::endl;
    std::cout << "total time of travel of Endeffector (t_f) = " << t_f << std::endl;


    //check which profile we need---------------------------------------------------------------------------------------
    bool trapez_profile = false;
    double t_m;
    if( t_f > (t_c*2) ){
        trapez_profile = true;
        std::cout << "t_f > t_c*2   => We need trapezoidal profile " << std::endl;
        // t_f for trapezoidal
        t_f = t_c + (total_distance / v_max);
        std::cout << "t_f  = " << t_f << std::endl;
    }else{
        trapez_profile = false;

        std::cout << "t_f <= t_c*2  => We need max_vel_profile " << std::endl;
        t_m = sqrt(abs(total_distance) / a_max);
        // t_f for max_vel_profile needs to be calculated
        t_f = 2*t_m;
        std::cout << "t_f  = " << t_f << std::endl;

    }

    //-----------------------calculate direction from Start- to Endpoint and normalize the vector-----------------------
    Vector<double, 3> path_dir_vec;

    double norm = (end_pos_vec - start_pos_vec).length();
    path_dir_vec = (end_pos_vec - start_pos_vec);
    path_dir_vec *= (1.0/norm);

    std::cout << "Direction Vector of path = " << std::endl;
    path_dir_vec.output();
    //------------------------------------------------------------------------------------------------------------------

    //Calculate points for Endeffector along linear path ---------------------------------------------------------------
    std::array<double,3> t_pos_vec{};

   // std::array<double,3> t_ori_vec{};
//    t_pos_vec[0] = -0.718376830;
//    t_pos_vec[1] = 1.89742719;
//    t_pos_vec[2] = 0.232490439;
//
//    t_ori_vec[0] = 2.664878255;
//    t_ori_vec[1] = -0.962452349;
//    t_ori_vec[2] = 2.87449;


    //get size of steps for orientation---------------------------------------------------------------------------------
    double number_steps = t_f*timesteps;
    double a_steps = a_diff/number_steps;
    double b_steps = b_diff/number_steps;
    double c_steps = c_diff/number_steps;

    //------------------------------------------------------------------------------------------------------------------


    std::vector<SixDPos*> positions;
//    double a = 0;
//    double b = 0;
//    double c = 0;
//   // double t = 0;
//    double x = 0;
//    double y = 0;
//    double z = 0;

    InvKinematics invKin;
    config_vec.push_back(new_start_cfg);
    std::vector<vector<Configuration*>*> nconf;

    std::vector<Configuration*>* temp_configs = new vector<Configuration*>();
    std::vector<Configuration*>* temp_configs2 = new vector<Configuration*>();
    int singularity = -1;

    for (int i = 0; i < t_f*timesteps; ++i) {

        double t = double(i)/timesteps;


        if(trapez_profile){
            t_pos_vec = trapezoidal_prof(start_pos_vec, end_pos_vec, t_f, t, t_c, a_max, v_max, path_dir_vec);
        }else{
            //implement a_max profile
            t_pos_vec = max_vel_profile(start_pos_vec, total_distance, t_f, t_m, t, path_dir_vec );
        }

        std::cout << "Point at time: " << t << std::endl;
        //t_pos_vec.output();
        std::cout << "Orientations at time: " << t << std::endl;
       // std::cout << "a:  " << a << " b:  " << b << " c:  " << c << std::endl;

        positions.push_back(new SixDPos(t_pos_vec[0], t_pos_vec[1], t_pos_vec[2], end_ori_vec[0] , end_ori_vec[1], end_ori_vec[2]));


        temp_configs2 = invKin.get_inv_kinematics(new SixDPos(t_pos_vec[0], t_pos_vec[1], t_pos_vec[2],
                                                             end_ori_vec[0] , end_ori_vec[1], end_ori_vec[2]));

        // check criteria for breaking for loop
        if (temp_configs2->size() == 0){
            std::cout << "Point at position: " << positions.at(i)->get_A() << ", " << positions.at(i)->get_B() << ", " << positions.at(i)->get_C() << "; is not reachable" << std::endl;
            break;
        }

        // catch singularity
        if( i != 0 && (temp_configs2->at(temp_configs2->size()-1)->get_configuration().operator[](3) == 0)
            && ((temp_configs2->at(temp_configs2->size()-1)->get_configuration().operator[](4) == 0))
            && ((temp_configs2->at(temp_configs2->size()-1)->get_configuration().operator[](5) == 0)) ) {
            // wrist singularity
            singularity = i;
        }else if(( i != 0 && (temp_configs2->at(temp_configs2->size()-1)->get_configuration().operator[](0) == 0) )){
            // shoulder singularity
            singularity = i;
        }else if(i != singularity + 1){
            // delete wrist sing variable to standard value
            singularity = -1;
        }

        if (singularity == -1 || i == singularity + 1){
            temp_configs = temp_configs2;
        }

        // catch special case that theta1 is between 175 and 180 or -175 and -180
        // atan cannot give values above 180, therefore we need to add additional configurations to the vector
        temp_configs = check_joint1_specialcase(temp_configs);


        // Selecting the right configurations:
        // StartPoint is a special case
        if(i == 0 && temp_configs->size() != 0){
            //write initial velocities and accelerations into vectors (all are 0)
            vector<double> vel_accel_temp;
            vel_accel_temp.reserve(6);
            for (int j = 0; j < 6; ++j) {
                vel_accel_temp.push_back(0);
            }
            joint_velocities_vec.push_back(vel_accel_temp);

            // get closest config while checking the limits
            int best_config = get_best_config(temp_configs, a_max_limit, joint_max_vel, timesteps, true);

            // check if there was a config that was not exceeding our robot limits
            if(best_config != -1){
                std::cout << "The best configuration at t: " << t << " is: " << best_config << std::endl;
                // add best config to configuration vector
                config_vec.push_back(temp_configs->at(best_config));
            }else{
                std::cout << "At t: " << t << " the next config is not in range of the limits." <<  std::endl;

                // insert function that determines new configurations between two configurations that are not reachable yet (because of limits)
                // first determine closest configuration (this time without checking the limits)
                best_config = get_best_config(temp_configs, a_max_limit, joint_max_vel, timesteps, false);

                if (movement_case != 2){
                    // here we check the limits of the chosen configuration (middle config gets pushed into config_vec inside the function itself)
                    num_iterations = 0;
                    num_iterations = add_middle_cfg(config_vec.at(config_vec.size()-1), temp_configs->at(best_config),
                                                    &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max_limit, joint_max_vel, timesteps, num_iterations);
                    if (num_iterations >= 10) {
                        // catch case if we use the recursive function more than 5 times
                        // this is so we don't end up with an infinite loop
                        std::perror("Stopped process of determining new configurations between points due to too many iterations");
                        break;
                    }
                }
            }

        }
        // Rest of the points
        if(i > 0 && temp_configs->size() != 0){
            int best_config = get_best_config(temp_configs, a_max_limit, joint_max_vel, timesteps, true);

            // check if there was a config that was not exceeding our robot limits
            if(best_config != -1){
                std::cout << "The best configuration at t: " << t << " is: " << best_config << std::endl;
                // add best config to configuration vector
                config_vec.push_back(temp_configs->at(best_config));
            }else{
                std::cout << "At t: " << t << " the next config is not in range of the limits." <<  std::endl;

                // insert function that determines new configurations between two configurations that are not reachable yet (because of limits)
                // first determine closest configuration (this time without checking the limits)
                best_config = get_best_config(temp_configs, a_max_limit, joint_max_vel, timesteps, false);

                // check if one of the distances is too large (changing from forward to backward case for example)
                std::vector<double> temp = calc_config_difference_to_vec(config_vec.at(config_vec.size()-1), temp_configs->at(best_config));
                for (int j = 0; j < 6; ++j) {
                    if((temp.at(j) > (config_switch_limit * M_PII/180)) && config_switch_indent == -1 ){
                        config_switch_indent = config_vec.size()-1;
                    }
                }
                if (config_switch_indent != -1){
                    std::cout << "Configuration Switch at position: ";
                    std::cout << positions.at(0) << ", " << positions.at(1) << ", " <<positions.at(2) << ", " << positions.at(3)
                    << ", " << positions.at(4) << ", " <<positions.at(5) << "." << std::endl;
                    std::cout << "We stop the planning of trajectory here. The robot will not move further." << std::endl;
                    break;
                }

                if (movement_case != 2){
                    // here we check the limits of the chosen configuration (middle config gets pushed into config_vec inside the function itself)
                    num_iterations = 0;
                    num_iterations = add_middle_cfg(config_vec.at(config_vec.size()-1), temp_configs->at(best_config),
                                    &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max_limit, joint_max_vel, timesteps, num_iterations);
                    if (num_iterations >= 10) {
                        // catch case if we use the recursive function more than 5 times
                        // this is so we don't end up with an infinite loop
                        std::perror("Stopped process of determining new configurations between points due to too many iterations");
                        break;
                    }
                }
            }
        }

    }

    std::cout << "number of points " << positions.size() << "number of configs " << config_vec.size() << std::endl;
    trajectory->set_trajectory(config_vec);
    //Dummy trajectory
//    trajectory->set_trajectory({
//                                       new Configuration({1.9073486328125e-06,-1.5707794427872,1.5707956552505,2.0742416381836e-05,9.5367431640625e-07,4.7683715820313e-07}),new Configuration({0.010499380528927,-1.5712878704071,1.5588240623474,9.4405086201732e-06,0.0019740660209209,-0.010497255250812}),new Configuration({0.020996853709221,-1.571796298027,1.5468524694443,-1.8613992551764e-06,0.0039471783675253,-0.020994987338781}),new Configuration({0.031494326889515,-1.572304725647,1.5348808765411,-1.3163306903152e-05,0.0059202904812992,-0.031492717564106}),new Configuration({0.041991800069809,-1.5728131532669,1.522909283638,-2.4465214664815e-05,0.0078934030607343,-0.041990451514721}),new Configuration({0.052489269524813,-1.5733215808868,1.5109376907349,-3.5767123335972e-05,0.0098665151745081,-0.052488181740046}),new Configuration({0.062986746430397,-1.5738300085068,1.4989660978317,-4.706903200713e-05,0.011839627288282,-0.06298591196537}),new Configuration({0.073484219610691,-1.5743384361267,1.4869945049286,-5.8370937040308e-05,0.013812739402056,-0.073483645915985}),new Configuration({0.083981692790985,-1.5748468637466,1.4750229120255,-6.9672845711466e-05,0.015785852447152,-0.0839813798666}),new Configuration({0.094479158520699,-1.5753552913666,1.4630514383316,-8.0974750744645e-05,0.017758963629603,-0.094479113817215}),new Configuration({0.10497663170099,-1.5758637189865,1.4510798454285,-9.2276663053781e-05,0.0197320766747,-0.10497684031725}),new Configuration({0.11547410488129,-1.5763721466064,1.4391082525253,-0.00010357856808696,0.021705189719796,-0.11547457426786}),new Configuration({0.12597158551216,-1.5768805742264,1.4271366596222,-0.0001148804803961,0.023678300902247,-0.1259723007679}),new Configuration({0.13646905124187,-1.5773890018463,1.4151650667191,-0.00012618237815332,0.025651413947344,-0.13647003471851}),new Configuration({0.14696653187275,-1.5778974294662,1.4031934738159,-0.00013748429773841,0.027624525129795,-0.14696776866913}),new Configuration({0.15746399760246,-1.5784057378769,1.3912218809128,-0.00014878620277159,0.029597638174891,-0.15746550261974}),new Configuration({0.16796147823334,-1.5789141654968,1.3792502880096,-0.00016008810780477,0.031570751219988,-0.16796323657036}),new Configuration({0.17845894396305,-1.5794225931168,1.3672786951065,-0.00017139001283795,0.033543862402439,-0.17846097052097}),new Configuration({0.18895640969276,-1.5799310207367,1.3553071022034,-0.00018269191787113,0.03551697358489,-0.18895870447159}),new Configuration({0.19945389032364,-1.5804394483566,1.3433355093002,-0.00019399383745622,0.037490088492632,-0.19945642352104}),new Configuration({0.20995135605335,-1.5809478759766,1.3313639163971,-0.0002052957424894,0.039463199675083,-0.20995415747166}),new Configuration({0.22044883668423,-1.5814563035965,1.319392323494,-0.00021659764752258,0.041436310857534,-0.22045189142227}),new Configuration({0.23094630241394,-1.5819647312164,1.3074207305908,-0.00022789955255575,0.043409425765276,-0.23094962537289}),new Configuration({0.24144378304482,-1.5824731588364,1.2954491376877,-0.00023920145758893,0.045382536947727,-0.2414473593235}),new Configuration({0.25194126367569,-1.5829815864563,1.2834775447845,-0.00025050336262211,0.047355648130178,-0.25194507837296}),new Configuration({0.26243871450424,-1.5834900140762,1.2715060710907,-0.00026180528220721,0.04932875931263,-0.26244282722473}),new Configuration({0.27293619513512,-1.5839984416962,1.2595344781876,-0.00027310717268847,0.051301874220371,-0.27294054627419}),new Configuration({0.28343367576599,-1.5845068693161,1.2475628852844,-0.00028440909227356,0.053274985402822,-0.28343829512596}),new Configuration({0.29393115639687,-1.585015296936,1.2355912923813,-0.00029571101185866,0.055248096585274,-0.29393601417542}),new Configuration({0.30442860722542,-1.585523724556,1.2236196994781,-0.00030701290233992,0.057221211493015,-0.30443376302719}),new Configuration({0.31492608785629,-1.5860321521759,1.211648106575,-0.00031831482192501,0.059194322675467,-0.31493148207664}),new Configuration({0.32542356848717,-1.5865405797958,1.1996765136719,-0.00032961671240628,0.061167433857918,-0.3254292011261}),new Configuration({0.33592104911804,-1.5870490074158,1.1877049207687,-0.00034091863199137,0.063140548765659,-0.33592694997787}),new Configuration({0.34641849994659,-1.5875574350357,1.1757333278656,-0.00035222055157647,0.06511365622282,-0.34642466902733}),new Configuration({0.35691598057747,-1.5880658626556,1.1637617349625,-0.00036352244205773,0.067086771130562,-0.3569224178791}),new Configuration({0.36741346120834,-1.5885742902756,1.1517901420593,-0.00037482436164282,0.069059886038303,-0.36742013692856}),new Configuration({0.3779109120369,-1.5890827178955,1.1398185491562,-0.00038612625212409,0.071032993495464,-0.37791788578033}),new Configuration({0.38840839266777,-1.5895911455154,1.1278469562531,-0.00039742817170918,0.073006108403206,-0.38841560482979}),new Configuration({0.39890587329865,-1.5900995731354,1.1158753633499,-0.00040873009129427,0.074979223310947,-0.39891332387924}),new Configuration({0.40940335392952,-1.5906080007553,1.1039037704468,-0.00042003198177554,0.076952330768108,-0.40941107273102}),new Configuration({0.41990080475807,-1.5911164283752,1.0919321775436,-0.00043133390136063,0.07892544567585,-0.41990879178047}),new Configuration({0.43039828538895,-1.5916248559952,1.0799605846405,-0.00044263579184189,0.080898560583591,-0.43040654063225}),new Configuration({0.44089576601982,-1.5921332836151,1.0679891109467,-0.00045393771142699,0.082871668040752,-0.4409042596817}),new Configuration({0.4513932466507,-1.592641711235,1.0560175180435,-0.00046523963101208,0.084844782948494,-0.45140200853348}),new Configuration({0.46189069747925,-1.593150138855,1.0440459251404,-0.00047654152149335,0.086817897856236,-0.46189972758293}),new Configuration({0.47238817811012,-1.5936584472656,1.0320743322372,-0.00048784344107844,0.088791005313396,-0.47239744663239}),new Configuration({0.482885658741,-1.5941668748856,1.0201027393341,-0.00049914536066353,0.090764120221138,-0.48289519548416}),new Configuration({0.49338313937187,-1.5946753025055,1.008131146431,-0.0005104472511448,0.092737227678299,-0.49339291453362}),new Configuration({0.50388062000275,-1.5951837301254,0.99615955352783,-0.00052174914162606,0.09471034258604,-0.50389063358307}),new Configuration({0.5143780708313,-1.5956921577454,0.98418796062469,-0.00053305109031498,0.096683457493782,-0.51438838243484}),new Configuration({0.52487552165985,-1.5962005853653,0.97221636772156,-0.00054435298079625,0.098656564950943,-0.52488613128662}),new Configuration({0.53537303209305,-1.5967090129852,0.96024477481842,-0.00055565487127751,0.10062967985868,-0.5353838801384}),new Configuration({0.5458704829216,-1.5972174406052,0.94827318191528,-0.00056695676175877,0.10260279476643,-0.54588156938553}),new Configuration({0.55636793375015,-1.5977258682251,0.93630158901215,-0.0005782587104477,0.10457590222359,-0.5563793182373}),new Configuration({0.56686544418335,-1.598234295845,0.92433005571365,-0.00058956060092896,0.10654901713133,-0.56687706708908}),new Configuration({0.5773628950119,-1.598742723465,0.91235846281052,-0.00060086249141023,0.10852213203907,-0.57737475633621}),new Configuration({0.5878604054451,-1.5992511510849,0.90038686990738,-0.00061216444009915,0.11049523949623,-0.58787250518799}),new Configuration({0.59835785627365,-1.5997595787048,0.88841527700424,-0.00062346633058041,0.11246835440397,-0.59837025403976}),new Configuration({0.6088553071022,-1.6002680063248,0.8764436841011,-0.00063476822106168,0.11444146931171,-0.60886800289154}),new Configuration({0.6193528175354,-1.6007764339447,0.86447209119797,-0.0006460701697506,0.11641457676888,-0.61936569213867}),new Configuration({0.62985026836395,-1.6012848615646,0.85250049829483,-0.00065737206023186,0.11838769167662,-0.62986344099045}),new Configuration({0.6403477191925,-1.6017932891846,0.84052890539169,-0.00066867395071313,0.12036080658436,-0.64036118984222}),new Configuration({0.6508452296257,-1.6023017168045,0.82855731248856,-0.00067997584119439,0.12233391404152,-0.65085887908936}),new Configuration({0.66134268045425,-1.6028101444244,0.81658577919006,-0.00069127778988332,0.12430702894926,-0.66135662794113}),new Configuration({0.67184019088745,-1.6033185720444,0.80461418628693,-0.00070257968036458,0.126280143857,-0.67185437679291}),new Configuration({0.682337641716,-1.6038269996643,0.79264259338379,-0.00071388157084584,0.12825325131416,-0.68235212564468}),new Configuration({0.69283509254456,-1.6043354272842,0.78067100048065,-0.00072518351953477,0.13022635877132,-0.69284981489182}),new Configuration({0.70333260297775,-1.6048438549042,0.76869940757751,-0.00073648541001603,0.13219948112965,-0.70334756374359}),new Configuration({0.7138300538063,-1.6053522825241,0.75672781467438,-0.00074778730049729,0.13417258858681,-0.71384531259537}),new Configuration({0.72432750463486,-1.605860710144,0.74475622177124,-0.00075908924918622,0.13614569604397,-0.7243430018425}),new Configuration({0.73482501506805,-1.606369137764,0.7327846288681,-0.00077039113966748,0.13811881840229,-0.73484075069427}),new Configuration({0.74532246589661,-1.6068775653839,0.72081309556961,-0.00078169303014874,0.14009192585945,-0.74533849954605}),new Configuration({0.75581991672516,-1.6073859930038,0.70884150266647,-0.00079299492063001,0.14206503331661,-0.75583624839783}),new Configuration({0.76631742715836,-1.6078944206238,0.69686990976334,-0.00080429686931893,0.14403815567493,-0.76633393764496}),new Configuration({0.77681487798691,-1.6084028482437,0.6848983168602,-0.0008155987598002,0.1460112631321,-0.77683168649673}),new Configuration({0.7873123884201,-1.6089111566544,0.67292672395706,-0.00082690065028146,0.14798437058926,-0.78732943534851}),new Configuration({0.79780983924866,-1.6094195842743,0.66095513105392,-0.00083820259897038,0.14995749294758,-0.79782712459564}),new Configuration({0.80830729007721,-1.6099280118942,0.64898353815079,-0.00084950448945165,0.15193060040474,-0.80832487344742}),new Configuration({0.81880480051041,-1.6104364395142,0.63701194524765,-0.00086080637993291,0.1539037078619,-0.81882262229919}),new Configuration({0.82930225133896,-1.6109448671341,0.62504041194916,-0.00087210832862183,0.15587683022022,-0.82932037115097}),new Configuration({0.83979970216751,-1.611453294754,0.61306881904602,-0.0008834102191031,0.15784993767738,-0.8398180603981}),new Configuration({0.85029721260071,-1.611961722374,0.60109722614288,-0.00089471210958436,0.15982304513454,-0.85031580924988}),new Configuration({0.86079466342926,-1.6124701499939,0.58912563323975,-0.00090601400006562,0.16179616749287,-0.86081355810165}),new Configuration({0.87129217386246,-1.6129785776138,0.57715404033661,-0.00091731594875455,0.16376927495003,-0.87131124734879}),new Configuration({0.88178962469101,-1.6134870052338,0.56518244743347,-0.00092861783923581,0.16574238240719,-0.88180899620056}),new Configuration({0.89228707551956,-1.6139954328537,0.55321085453033,-0.00093991972971708,0.16771550476551,-0.89230674505234}),new Configuration({0.90278458595276,-1.6145038604736,0.5412392616272,-0.000951221678406,0.16968861222267,-0.90280449390411}),new Configuration({0.91328203678131,-1.6150122880936,0.52926766872406,-0.00096252356888726,0.17166171967983,-0.91330218315125}),new Configuration({0.92377948760986,-1.6155207157135,0.51729613542557,-0.00097382545936853,0.17363484203815,-0.92379993200302}),new Configuration({0.93427699804306,-1.6160291433334,0.50532454252243,-0.00098512740805745,0.17560794949532,-0.9342976808548}),new Configuration({0.94477444887161,-1.6165375709534,0.49335294961929,-0.00099642924033105,0.17758105695248,-0.94479537010193}),new Configuration({0.95527189970016,-1.6170459985733,0.48138135671616,-0.00100773118902,0.1795541793108,-0.9552931189537}),new Configuration({0.96576941013336,-1.6175544261932,0.46940976381302,-0.0010190331377089,0.18152728676796,-0.96579086780548}),new Configuration({0.97626686096191,-1.6180628538132,0.45743817090988,-0.0010303349699825,0.18350039422512,-0.97628861665726}),new Configuration({0.98676437139511,-1.6185712814331,0.44546660780907,-0.0010416369186714,0.18547350168228,-0.98678630590439}),new Configuration({0.99726182222366,-1.619079709053,0.43349501490593,-0.0010529388673604,0.1874466240406,-0.99728405475616}),new Configuration({1.0077593326569,-1.619588136673,0.42152342200279,-0.001064240699634,0.18941973149776,-1.0077817440033}),new Configuration({1.0182567834854,-1.6200965642929,0.40955182909966,-0.0010755426483229,0.19139283895493,-1.0182795524597}),new Configuration({1.028754234314,-1.6206049919128,0.39758026599884,-0.0010868445970118,0.19336596131325,-1.0287772417068}),new Configuration({1.0392516851425,-1.6211134195328,0.3856086730957,-0.0010981464292854,0.19533906877041,-1.039274930954}),new Configuration({1.0497491359711,-1.6216218471527,0.37363708019257,-0.0011094483779743,0.19731217622757,-1.0497727394104}),new Configuration({1.0602467060089,-1.6221302747726,0.36166548728943,-0.0011207503266633,0.19928529858589,-1.0602704286575}),new Configuration({1.0707441568375,-1.6226387023926,0.34969392418861,-0.0011320521589369,0.20125840604305,-1.070768237114}),new Configuration({1.081241607666,-1.6231471300125,0.33772233128548,-0.0011433541076258,0.20323151350021,-1.0812659263611}),new Configuration({1.0917390584946,-1.6236555576324,0.32575073838234,-0.0011546559398994,0.20520463585854,-1.0917636156082}),new Configuration({1.1022365093231,-1.6241638660431,0.3137791454792,-0.0011659578885883,0.2071777433157,-1.1022614240646}),new Configuration({1.1127339601517,-1.624672293663,0.30180758237839,-0.0011772598372772,0.20915085077286,-1.1127591133118}),new Configuration({1.1232315301895,-1.625180721283,0.28983598947525,-0.0011885616695508,0.21112397313118,-1.1232568025589}),new Configuration({1.1337289810181,-1.6256891489029,0.27786439657211,-0.0011998636182398,0.21309708058834,-1.1337546110153}),new Configuration({1.1442264318466,-1.6261975765228,0.26589280366898,-0.0012111655669287,0.2150701880455,-1.1442523002625}),new Configuration({1.1547238826752,-1.6267060041428,0.25392121076584,-0.0012224673992023,0.21704331040382,-1.1547499895096}),new Configuration({1.1652213335037,-1.6272144317627,0.24194963276386,-0.0012337693478912,0.21901641786098,-1.165247797966}),new Configuration({1.1757189035416,-1.6277228593826,0.22997805476189,-0.0012450712965801,0.22098952531815,-1.1757454872131}),new Configuration({1.1862163543701,-1.6282312870026,0.21800646185875,-0.0012563731288537,0.22296264767647,-1.1862431764603}),new Configuration({1.1967138051987,-1.6287397146225,0.20603488385677,-0.0012676750775427,0.22493575513363,-1.1967409849167}),new Configuration({1.2072112560272,-1.6292481422424,0.19406329095364,-0.0012789770262316,0.22690886259079,-1.2072386741638}),new Configuration({1.2177087068558,-1.6297565698624,0.18209171295166,-0.0012902788585052,0.22888198494911,-1.2177364826202}),new Configuration({1.2282062768936,-1.6302649974823,0.17012012004852,-0.0013015808071941,0.23085509240627,-1.2282341718674}),new Configuration({1.2387037277222,-1.6307734251022,0.15814854204655,-0.001312882755883,0.23282819986343,-1.2387318611145}),new Configuration({1.2492011785507,-1.6312818527222,0.14617694914341,-0.0013241845881566,0.23480132222176,-1.2492296695709}),new Configuration({1.2596986293793,-1.6317902803421,0.13420537114143,-0.0013354865368456,0.23677442967892,-1.2597273588181}),new Configuration({1.2701960802078,-1.632298707962,0.12223378568888,-0.0013467884855345,0.23874753713608,-1.2702250480652}),new Configuration({1.2806935310364,-1.632807135582,0.11026220023632,-0.0013580903178081,0.2407206594944,-1.2807228565216}),new Configuration({1.2911911010742,-1.6333155632019,0.098290607333183,-0.001369392266497,0.24269376695156,-1.2912205457687}),new Configuration({1.3016885519028,-1.6338239908218,0.086319021880627,-0.0013806940987706,0.24466687440872,-1.3017182350159}),new Configuration({1.3121860027313,-1.6343324184418,0.07434743642807,-0.0013919960474595,0.24663999676704,-1.3122160434723}),new Configuration({1.3226834535599,-1.6348408460617,0.062375854700804,-0.0014032979961485,0.24861310422421,-1.3227137327194}),new Configuration({1.3331809043884,-1.6353492736816,0.050404269248247,-0.0014145998284221,0.25058621168137,-1.3332114219666}),new Configuration({1.3436784744263,-1.6358577013016,0.038432683795691,-0.001425901777111,0.25255933403969,-1.343709230423}),new Configuration({1.3541759252548,-1.6363661289215,0.026461096480489,-0.0014372037257999,0.25453242659569,-1.3542069196701}),new Configuration({1.3646733760834,-1.6368745565414,0.014489511027932,-0.0014485055580735,0.25650554895401,-1.3647047281265}),new Configuration({1.3751708269119,-1.6373829841614,0.0025179251097143,-0.0014598075067624,0.25847867131233,-1.3752024173737}),new Configuration({1.3856682777405,-1.6378914117813,-0.0094536608085036,-0.0014711094554514,0.26045176386833,-1.3857001066208}),new Configuration({1.396165728569,-1.6383998394012,-0.021425247192383,-0.001482411287725,0.26242488622665,-1.3961979150772}),new Configuration({1.4066632986069,-1.6389082670212,-0.033396832644939,-0.0014937132364139,0.26439800858498,-1.4066956043243}),new Configuration({1.4171607494354,-1.6394165754318,-0.045368418097496,-0.0015050151851028,0.26637110114098,-1.4171932935715}),new Configuration({1.427658200264,-1.6399250030518,-0.057340003550053,-0.0015163170173764,0.2683442234993,-1.4276911020279}),new Configuration({1.4381556510925,-1.6404334306717,-0.069311589002609,-0.0015276189660653,0.27031734585762,-1.438188791275}),new Configuration({1.4486531019211,-1.6409418582916,-0.081283174455166,-0.0015389209147543,0.27229043841362,-1.4486864805222}),new Configuration({1.4591506719589,-1.6414502859116,-0.093254759907722,-0.0015502227470279,0.27426356077194,-1.4591842889786}),new Configuration({1.4696481227875,-1.6419587135315,-0.10522634536028,-0.0015615246957168,0.27623668313026,-1.4696819782257}),new Configuration({1.480145573616,-1.6424671411514,-0.11719793081284,-0.0015728265279904,0.27820977568626,-1.4801796674728}),new Configuration({1.4906430244446,-1.6429755687714,-0.12916952371597,-0.0015841284766793,0.28018289804459,-1.4906774759293}),new Configuration({1.5011404752731,-1.6434839963913,-0.14114110171795,-0.0015954304253682,0.28215602040291,-1.5011751651764}),new Configuration({1.5116379261017,-1.6439924240112,-0.15311269462109,-0.0016067322576419,0.28412911295891,-1.5116729736328}),new Configuration({1.5221354961395,-1.6445008516312,-0.16508427262306,-0.0016180342063308,0.28610223531723,-1.5221706628799}),new Configuration({1.5326329469681,-1.6450092792511,-0.1770558655262,-0.0016293361550197,0.28807535767555,-1.5326683521271}),new Configuration({1.5431303977966,-1.645517706871,-0.18902744352818,-0.0016406379872933,0.29004845023155,-1.5431661605835}),new Configuration({1.5536278486252,-1.646026134491,-0.20099903643131,-0.0016519399359822,0.29202157258987,-1.5536638498306}),new Configuration({1.5641252994537,-1.6465345621109,-0.21297061443329,-0.0016632418846712,0.2939946949482,-1.5641615390778}),new Configuration({1.5746228694916,-1.6470429897308,-0.22494220733643,-0.0016745437169448,0.2959677875042,-1.5746593475342}),new Configuration({1.5851203203201,-1.6475514173508,-0.2369137853384,-0.0016858456656337,0.29794090986252,-1.5851570367813}),new Configuration({1.5956177711487,-1.6480598449707,-0.24888537824154,-0.0016971476143226,0.29991403222084,-1.5956547260284}),new Configuration({1.6061152219772,-1.6485682725906,-0.26085695624352,-0.0017084494465962,0.30188712477684,-1.6061525344849}),new Configuration({1.6166126728058,-1.6490767002106,-0.27282854914665,-0.0017197513952851,0.30386024713516,-1.616650223732}),new Configuration({1.6271102428436,-1.6495851278305,-0.28480014204979,-0.0017310533439741,0.30583336949348,-1.6271479129791}),new Configuration({1.6376076936722,-1.6500935554504,-0.2967717051506,-0.0017423551762477,0.30780646204948,-1.6376457214355}),new Configuration({1.6481051445007,-1.6506019830704,-0.30874329805374,-0.0017536571249366,0.30977958440781,-1.6481434106827}),new Configuration({1.6586025953293,-1.6511104106903,-0.32071489095688,-0.0017649590736255,0.31175270676613,-1.6586412191391}),new Configuration({1.6691000461578,-1.6516188383102,-0.33268648386002,-0.0017762609058991,0.31372579932213,-1.6691389083862}),new Configuration({1.6795974969864,-1.6521272659302,-0.34465804696083,-0.001787562854588,0.31569892168045,-1.6796365976334}),new Configuration({1.6900950670242,-1.6526356935501,-0.35662963986397,-0.0017988646868616,0.31767204403877,-1.6901344060898}),new Configuration({1.7005925178528,-1.65314412117,-0.36860123276711,-0.0018101666355506,0.31964513659477,-1.7006320953369}),new Configuration({1.7110899686813,-1.65365254879,-0.38057282567024,-0.0018214685842395,0.32161825895309,-1.711129784584}),new Configuration({1.7215874195099,-1.6541609764099,-0.39254441857338,-0.0018327704165131,0.32359138131142,-1.7216275930405}),new Configuration({1.7320848703384,-1.6546692848206,-0.40451598167419,-0.001844072365202,0.32556447386742,-1.7321252822876}),new Configuration({1.7425824403763,-1.6551777124405,-0.41648757457733,-0.0018553743138909,0.32753759622574,-1.7426229715347}),new Configuration({1.7530798912048,-1.6556861400604,-0.42845916748047,-0.0018666761461645,0.32951071858406,-1.7531207799911}),new Configuration({1.7635773420334,-1.6561945676804,-0.44043076038361,-0.0018779780948535,0.33148381114006,-1.7636184692383}),new Configuration({1.7740747928619,-1.6567029953003,-0.45240232348442,-0.0018892800435424,0.33345693349838,-1.7741161584854}),new Configuration({1.7845722436905,-1.6572114229202,-0.46437391638756,-0.001900581875816,0.3354300558567,-1.7846139669418}),new Configuration({1.795069694519,-1.6577198505402,-0.4763455092907,-0.0019118838245049,0.3374031484127,-1.795111656189}),new Configuration({1.8055672645569,-1.6582282781601,-0.48831710219383,-0.0019231857731938,0.33937627077103,-1.8056094646454}),new Configuration({1.8160647153854,-1.65873670578,-0.50028866529465,-0.0019344876054674,0.34134939312935,-1.8161071538925}),new Configuration({1.826562166214,-1.6592451334,-0.51226025819778,-0.0019457895541564,0.34332248568535,-1.8266048431396}),new Configuration({1.8370596170425,-1.6597535610199,-0.52423185110092,-0.0019570915028453,0.34529560804367,-1.8371026515961}),new Configuration({1.8475570678711,-1.6602619886398,-0.53620344400406,-0.0019683933351189,0.34726873040199,-1.8476003408432}),new Configuration({1.8580546379089,-1.6607704162598,-0.5481750369072,-0.0019796951673925,0.34924182295799,-1.8580980300903}),new Configuration({1.8685520887375,-1.6612788438797,-0.56014662981033,-0.0019909972324967,0.35121494531631,-1.8685958385468}),new Configuration({1.879049539566,-1.6617872714996,-0.57211816310883,-0.0020022990647703,0.35318806767464,-1.8790935277939}),new Configuration({1.8895469903946,-1.6622956991196,-0.58408975601196,-0.0020136008970439,0.35516116023064,-1.889591217041}),new Configuration({1.9000444412231,-1.6628041267395,-0.5960613489151,-0.0020249029621482,0.35713428258896,-1.9000890254974}),new Configuration({1.9105418920517,-1.6633125543594,-0.60803294181824,-0.0020362047944218,0.35910740494728,-1.9105867147446}),new Configuration({1.9210394620895,-1.6638209819794,-0.62000453472137,-0.0020475066266954,0.36108049750328,-1.9210844039917}),new Configuration({1.9315369129181,-1.6643294095993,-0.63197612762451,-0.0020588086917996,0.3630536198616,-1.9315822124481}),new Configuration({1.9420343637466,-1.6648378372192,-0.64394772052765,-0.0020701105240732,0.36502674221992,-1.9420799016953}),new Configuration({1.9525318145752,-1.6653462648392,-0.65591931343079,-0.0020814123563468,0.36699983477592,-1.9525777101517}),new Configuration({1.9630292654037,-1.6658546924591,-0.66789084672928,-0.0020927144214511,0.36897295713425,-1.9630753993988}),new Configuration({1.9735268354416,-1.666363120079,-0.67986243963242,-0.0021040162537247,0.37094604969025,-1.9735730886459}),new Configuration({1.9840242862701,-1.666871547699,-0.69183403253555,-0.0021153180859983,0.37291917204857,-1.9840708971024}),new Configuration({1.9945217370987,-1.6673799753189,-0.70380562543869,-0.0021266201511025,0.37489229440689,-1.9945685863495}),new Configuration({2.0050191879272,-1.6678884029388,-0.71577721834183,-0.0021379219833761,0.37686538696289,-2.0050663948059}),new Configuration({2.0155167579651,-1.6683968305588,-0.72774881124496,-0.0021492238156497,0.37883850932121,-2.0155639648438}),new Configuration({2.0260140895844,-1.6689052581787,-0.7397204041481,-0.002160525880754,0.38081163167953,-2.0260617733002}),new Configuration({2.0365116596222,-1.6694136857986,-0.75169199705124,-0.0021718277130276,0.38278472423553,-2.0365595817566}),new Configuration({2.0470089912415,-1.6699219942093,-0.76366358995438,-0.0021831295453012,0.38475784659386,-2.0470571517944}),new Configuration({2.0575065612793,-1.6704304218292,-0.77563512325287,-0.0021944316104054,0.38673096895218,-2.0575549602509}),new Configuration({2.0680041313171,-1.6709388494492,-0.78760671615601,-0.002205733442679,0.38870406150818,-2.0680527687073}),new Configuration({2.0785014629364,-1.6714472770691,-0.79957830905914,-0.0022170352749527,0.3906771838665,-2.0785503387451}),new Configuration({2.0889990329742,-1.671955704689,-0.81154990196228,-0.0022283373400569,0.39265030622482,-2.0890481472015}),new Configuration({2.0994963645935,-1.672464132309,-0.82352149486542,-0.0022396391723305,0.39462339878082,-2.099545955658}),new Configuration({2.1099939346313,-1.6729725599289,-0.83549308776855,-0.0022509410046041,0.39659652113914,-2.1100435256958}),new Configuration({2.1204915046692,-1.6734809875488,-0.84746468067169,-0.0022622430697083,0.39856964349747,-2.1205413341522}),new Configuration({2.1309888362885,-1.6739894151688,-0.85943627357483,-0.0022735449019819,0.40054273605347,-2.1310391426086}),new Configuration({2.1414864063263,-1.6744978427887,-0.87140780687332,-0.0022848467342556,0.40251585841179,-2.1415369510651}),new Configuration({2.1519837379456,-1.6750062704086,-0.88337939977646,-0.0022961487993598,0.40448898077011,-2.1520345211029}),new Configuration({2.1624813079834,-1.6755146980286,-0.8953509926796,-0.0023074506316334,0.40646207332611,-2.1625323295593}),new Configuration({2.1729788780212,-1.6760231256485,-0.90732258558273,-0.002318752463907,0.40843519568443,-2.1730301380157}),new Configuration({2.1834762096405,-1.6765315532684,-0.91929417848587,-0.0023300542961806,0.41040831804276,-2.1835277080536}),new Configuration({2.1939737796783,-1.6770399808884,-0.93126577138901,-0.0023413563612849,0.41238141059875,-2.19402551651}),new Configuration({2.2044711112976,-1.6775484085083,-0.94323736429214,-0.0023526581935585,0.41435453295708,-2.2045233249664}),new Configuration({2.2149686813354,-1.6780568361282,-0.95520895719528,-0.0023639600258321,0.4163276553154,-2.2150208950043}),new Configuration({2.2254660129547,-1.6785652637482,-0.96718049049377,-0.0023752620909363,0.4183007478714,-2.2255187034607}),new Configuration({2.2359635829926,-1.6790736913681,-0.97915208339691,-0.0023865639232099,0.42027387022972,-2.2360165119171}),new Configuration({2.2464611530304,-1.679582118988,-0.99112367630005,-0.0023978657554835,0.42224699258804,-2.246514081955}),new Configuration({2.2569584846497,-1.680090546608,-1.0030952692032,-0.0024091678205878,0.42422008514404,-2.2570118904114}),new Configuration({2.2674560546875,-1.6805989742279,-1.0150668621063,-0.0024204696528614,0.42619320750237,-2.2675096988678}),new Configuration({2.2779533863068,-1.6811074018478,-1.0270384550095,-0.002431771485135,0.42816632986069,-2.2780072689056}),new Configuration({2.2884509563446,-1.6816158294678,-1.0390100479126,-0.0024430735502392,0.43013942241669,-2.2885050773621}),new Configuration({2.2989485263824,-1.6821242570877,-1.0509816408157,-0.0024543753825128,0.43211254477501,-2.2990028858185}),new Configuration({2.3094458580017,-1.6826326847076,-1.0629532337189,-0.0024656772147864,0.43408566713333,-2.3095004558563}),new Configuration({2.3199434280396,-1.6831411123276,-1.074924826622,-0.0024769792798907,0.43605875968933,-2.3199982643127}),new Configuration({2.3304407596588,-1.6836495399475,-1.0868964195251,-0.0024882811121643,0.43803188204765,-2.3304960727692}),new Configuration({2.3409383296967,-1.6841579675674,-1.0988680124283,-0.0024995829444379,0.44000500440598,-2.340993642807}),new Configuration({2.3514358997345,-1.6846663951874,-1.1108396053314,-0.0025108850095421,0.44197809696198,-2.3514914512634}),new Configuration({2.3619332313538,-1.685174703598,-1.1228110790253,-0.0025221868418157,0.4439512193203,-2.3619892597198}),new Configuration({2.3724308013916,-1.685683131218,-1.1347826719284,-0.0025334886740893,0.44592434167862,-2.3724868297577}),new Configuration({2.3829281330109,-1.6861915588379,-1.1467542648315,-0.0025447907391936,0.44789743423462,-2.3829846382141}),new Configuration({2.3934257030487,-1.6866999864578,-1.1587258577347,-0.0025560925714672,0.44987055659294,-2.3934824466705}),new Configuration({2.4039232730865,-1.6872084140778,-1.1706974506378,-0.0025673944037408,0.45184367895126,-2.4039800167084}),new Configuration({2.4144206047058,-1.6877168416977,-1.182669043541,-0.002578696468845,0.45381677150726,-2.4144778251648}),new Configuration({2.4249181747437,-1.6882252693176,-1.1946406364441,-0.0025899983011186,0.45578989386559,-2.4249756336212}),new Configuration({2.4354155063629,-1.6887336969376,-1.2066122293472,-0.0026013001333922,0.45776301622391,-2.4354734420776}),new Configuration({2.4459130764008,-1.6892421245575,-1.2185838222504,-0.0026126021984965,0.45973610877991,-2.4459710121155}),new Configuration({2.4564106464386,-1.6897505521774,-1.2305554151535,-0.0026239040307701,0.46170923113823,-2.4564688205719}),new Configuration({2.4669079780579,-1.6902589797974,-1.2425270080566,-0.0026352058630437,0.46368235349655,-2.4669666290283}),new Configuration({2.4774055480957,-1.6907674074173,-1.2544986009598,-0.0026465079281479,0.46565544605255,-2.4774641990662}),new Configuration({2.487902879715,-1.6912758350372,-1.2664701938629,-0.0026578097604215,0.46762856841087,-2.4879620075226}),new Configuration({2.4984004497528,-1.6917842626572,-1.2784417867661,-0.0026691115926951,0.4696016907692,-2.498459815979}),new Configuration({2.5088977813721,-1.6922926902771,-1.2904133796692,-0.0026804136577994,0.4715747833252,-2.5089573860168}),new Configuration({2.5193953514099,-1.692801117897,-1.3023849725723,-0.002691715490073,0.47354790568352,-2.5194551944733}),new Configuration({2.5298929214478,-1.693309545517,-1.3143564462662,-0.0027030173223466,0.47552102804184,-2.5299530029297}),new Configuration({2.540390253067,-1.6938179731369,-1.3263280391693,-0.0027143193874508,0.47749412059784,-2.5404505729675}),new Configuration({2.5508878231049,-1.6943264007568,-1.3382996320724,-0.0027256212197244,0.47946724295616,-2.550948381424}),new Configuration({2.5613851547241,-1.6948348283768,-1.3502712249756,-0.002736923051998,0.48144036531448,-2.5614461898804}),new Configuration({2.571882724762,-1.6953432559967,-1.3622428178787,-0.0027482248842716,0.48341345787048,-2.5719437599182}),new Configuration({2.5823802947998,-1.6958516836166,-1.3742144107819,-0.0027595269493759,0.48538658022881,-2.5824415683746}),new Configuration({2.5928776264191,-1.6963601112366,-1.386186003685,-0.0027708287816495,0.48735970258713,-2.5929393768311}),new Configuration({2.6033751964569,-1.6968685388565,-1.3981575965881,-0.0027821306139231,0.48933279514313,-2.6034369468689}),new Configuration({2.6138725280762,-1.6973769664764,-1.4101291894913,-0.0027934326790273,0.49130591750145,-2.6139347553253}),new Configuration({2.624370098114,-1.6978853940964,-1.4221007823944,-0.0028047345113009,0.49327903985977,-2.6244325637817}),new Configuration({2.6348676681519,-1.6983938217163,-1.4340723752975,-0.0028160363435745,0.49525213241577,-2.6349301338196}),new Configuration({2.6453649997711,-1.6989022493362,-1.4460439682007,-0.0028273384086788,0.49722525477409,-2.645427942276}),new Configuration({2.655862569809,-1.6994106769562,-1.4580155611038,-0.0028386402409524,0.49919837713242,-2.6559257507324}),new Configuration({2.6663599014282,-1.6999191045761,-1.469987154007,-0.002849942073226,0.50117146968842,-2.6664233207703}),new Configuration({2.6768574714661,-1.7004274129868,-1.4819587469101,-0.0028612441383302,0.50314456224442,-2.6769211292267}),new Configuration({2.6873550415039,-1.7009358406067,-1.4939303398132,-0.0028725459706038,0.50511771440506,-2.6874189376831}),new Configuration({2.6978523731232,-1.7014442682266,-1.5059019327164,-0.0028838478028774,0.50709080696106,-2.6979165077209}),new Configuration({2.708349943161,-1.7019526958466,-1.5178734064102,-0.0028951498679817,0.50906389951706,-2.7084143161774}),new Configuration({2.7188472747803,-1.7024611234665,-1.5298449993134,-0.0029064517002553,0.5110370516777,-2.7189121246338}),new Configuration({2.7293448448181,-1.7029695510864,-1.5418165922165,-0.0029177535325289,0.5130101442337,-2.7294099330902}),new Configuration({2.739842414856,-1.7034779787064,-1.5537881851196,-0.0029290555976331,0.5149832367897,-2.7399075031281}),new Configuration({2.7503397464752,-1.7039864063263,-1.5657597780228,-0.0029403574299067,0.51695638895035,-2.7504053115845}),new Configuration({2.7608373165131,-1.7044948339462,-1.5777313709259,-0.0029516592621803,0.51892948150635,-2.7609031200409}),new Configuration({2.7713346481323,-1.7050032615662,-1.589702963829,-0.0029629613272846,0.52090257406235,-2.7714006900787}),new Configuration({2.7818322181702,-1.7055116891861,-1.6016745567322,-0.0029742631595582,0.52287572622299,-2.7818984985352}),new Configuration({2.7923295497894,-1.706020116806,-1.6136461496353,-0.0029855649918318,0.52484881877899,-2.7923963069916}),new Configuration({2.8028271198273,-1.706528544426,-1.6256177425385,-0.002996867056936,0.52682191133499,-2.8028938770294}),new Configuration({2.8133246898651,-1.7070369720459,-1.6375893354416,-0.0030081688892096,0.52879506349564,-2.8133916854858}),new Configuration({2.8238220214844,-1.7075453996658,-1.6495609283447,-0.0030194707214832,0.53076815605164,-2.8238894939423}),new Configuration({2.8343195915222,-1.7080538272858,-1.6615325212479,-0.0030307727865875,0.53274124860764,-2.8343870639801}),new Configuration({2.8448169231415,-1.7085622549057,-1.673504114151,-0.0030420746188611,0.53471440076828,-2.8448848724365}),new Configuration({2.8553144931793,-1.7090706825256,-1.6854757070541,-0.0030533764511347,0.53668749332428,-2.8553826808929}),new Configuration({2.8658120632172,-1.7095791101456,-1.6974472999573,-0.0030646785162389,0.53866058588028,-2.8658802509308}),new Configuration({2.8763093948364,-1.7100875377655,-1.7094188928604,-0.0030759803485125,0.54063373804092,-2.8763780593872}),new Configuration({2.8868069648743,-1.7105959653854,-1.7213903665543,-0.0030872821807861,0.54260683059692,-2.8868758678436}),new Configuration({2.8973042964935,-1.7111043930054,-1.7333619594574,-0.0030985842458904,0.54457992315292,-2.8973734378815}),new Configuration({2.9078018665314,-1.7116128206253,-1.7453335523605,-0.003109886078164,0.54655307531357,-2.9078712463379}),new Configuration({2.9182994365692,-1.7121212482452,-1.7573051452637,-0.0031211879104376,0.54852616786957,-2.9183690547943}),new Configuration({2.9287967681885,-1.7126296758652,-1.7692767381668,-0.0031324899755418,0.55049926042557,-2.9288666248322}),new Configuration({2.9392943382263,-1.7131381034851,-1.7812483310699,-0.0031437918078154,0.55247241258621,-2.9393644332886}),new Configuration({2.9497916698456,-1.713646531105,-1.7932199239731,-0.003155093640089,0.55444550514221,-2.949862241745}),new Configuration({2.9602892398834,-1.714154958725,-1.8051915168762,-0.0031663954723626,0.55641859769821,-2.9603598117828}),new Configuration({2.9707868099213,-1.7146633863449,-1.8171631097794,-0.0031776975374669,0.55839174985886,-2.9708576202393}),new Configuration({2.9812841415405,-1.7151718139648,-1.8291347026825,-0.0031889993697405,0.56036484241486,-2.9813554286957}),new Configuration({2.9917817115784,-1.7156801223755,-1.8411062955856,-0.0032003012020141,0.56233793497086,-2.9918529987335}),new Configuration({3.0022790431976,-1.7161885499954,-1.8530778884888,-0.0032116032671183,0.5643110871315,-3.0023508071899}),new Configuration({3.0127766132355,-1.7166969776154,-1.8650494813919,-0.0032229050993919,0.5662841796875,-3.0128486156464}),new Configuration({3.0232739448547,-1.7172054052353,-1.877021074295,-0.0032342069316655,0.5682572722435,-3.0233464241028}),new Configuration({3.0337715148926,-1.7177138328552,-1.8889926671982,-0.0032455089967698,0.57023042440414,-3.0338439941406}),new Configuration({3.0442690849304,-1.7182222604752,-1.9009642601013,-0.0032568108290434,0.57220351696014,-3.044341802597}),new Configuration({3.0547664165497,-1.7187306880951,-1.9129357337952,-0.003268112661317,0.57417660951614,-3.0548396110535}),new Configuration({3.0652639865875,-1.719239115715,-1.9249073266983,-0.0032794147264212,0.57614976167679,-3.0653371810913}),new Configuration({3.0757613182068,-1.719747543335,-1.9368789196014,-0.0032907165586948,0.57812285423279,-3.0758349895477}),new Configuration({3.0862588882446,-1.7202559709549,-1.9488505125046,-0.0033020183909684,0.58009594678879,-3.0863327980042}),new Configuration({3.0967564582825,-1.7207643985748,-1.9608221054077,-0.0033133204560727,0.58206909894943,-3.096830368042}),new Configuration({3.1072537899017,-1.7212728261948,-1.9727936983109,-0.0033246222883463,0.58404219150543,-3.1073281764984}),new Configuration({3.1177513599396,-1.7217812538147,-1.984765291214,-0.0033359241206199,0.58601528406143,-3.1178259849548}),new Configuration({3.1282486915588,-1.7222896814346,-1.9967368841171,-0.0033472261857241,0.58798843622208,-3.1283235549927}),new Configuration({3.1387462615967,-1.7227981090546,-2.0087084770203,-0.0033585280179977,0.58996152877808,-3.1388213634491})
//                               });

    return trajectory;
}

int Lin::get_best_config(std::vector<Configuration*>* temp_configs, double a_max, Vector<double, 6> joint_max_vel, double timesteps, bool checklimits){
    std::vector<double> distances;
    double distance = 10000;
    int temp = -1;
    if (checklimits) {
        for (unsigned int j = 0; j < temp_configs->size(); ++j) {
            if (calc_config_difference(config_vec.at(config_vec.size() - 1), temp_configs->at(j)) < distance) {

                // check if new config at new timepoint exceeds our limits of v & a
//                bool yes = checkconfiglimits(config_vec.at(config_vec.size() - 1), temp_configs->at(j),
//                                  &joint_velocities_vec.at(joint_velocities_vec.size() - 1), a_max, joint_max_vel,
//                                  timesteps);
                if (checkconfiglimits(config_vec.at(config_vec.size() - 1), temp_configs->at(j),
                                      &joint_velocities_vec.at(joint_velocities_vec.size() - 1), a_max, joint_max_vel,
                                      timesteps, false)) {
                    distance = calc_config_difference(config_vec.at(config_vec.size() - 1), temp_configs->at(j));
                    temp = j;
                    std::cout << "distance of config " << j + 1 << " is smaller than previous one" << std::endl;
                } else {
                    std::cout << "distance of config " << j + 1 << " is smaller than previous one" << std::endl;
                    std::cout << "However the new config exceeds robot limits" << std::endl;
                }
            }

        }
        if (temp != -1) {
            // check limits of the best config to add the velocity values to the global vector
            checkconfiglimits(config_vec.at(config_vec.size() - 1), temp_configs->at(temp),
                              &joint_velocities_vec.at(joint_velocities_vec.size() - 1), a_max, joint_max_vel,
                              timesteps, true);
        }

    }else{
        for (unsigned int j = 0; j < temp_configs->size(); ++j) {
            if (calc_config_difference(config_vec.at(config_vec.size() - 1), temp_configs->at(j)) < distance) {
                // check if new config at new timepoint exceeds our limits of v & a

                distance = calc_config_difference(config_vec.at(config_vec.size() - 1), temp_configs->at(j));
                temp = j;
                std::cout << "distance of config " << j + 1 << " is smaller than previous one" << std::endl;
            }

        }
    }
    return temp;
}

//function that returns position of a joint at time t
std::array<double,3> Lin::max_vel_profile(Vector<double, 3> start_pos, double distance, double t_f, double t_m, double t,
                                       Vector<double, 3> path_dir_vec ) {
    std::array<double,3> t_pos{};
    //calculate t_m and t_f for max_vel_profile
    if (t < 0) {
        return t_pos;
    } else if (t < (t_f/2) ) {
        for (int i = 0; i < 3; ++i) {
            t_pos[i] = start_pos[i] + 2 * pow((t / t_f), 2) * distance * path_dir_vec[i];
        }
        //t_pos = start_pos + 2 * pow((t / t_f), 2) ;
        return t_pos;
    } else if ((t_m < t) && (t <= t_f)) {
        for (int i = 0; i < 3; ++i) {
            t_pos[i] = start_pos[i] + (-1 + 4 * (t / t_f) - 2 * pow((t / t_f), 2)) * distance * path_dir_vec[i];
        }
        // t_pos = start_pos + (-1 + 4 * (t / t_f) - 2 * pow((t / t_f), 2)) ;
        return t_pos;
    } else {
        return t_pos;     // no change to position
    }

}

std::array<double,3> Lin::trapezoidal_prof(Vector<double, 3> start_pos, Vector<double, 3> end_pos,
                                        double t_f, double t, double t_c, double a_max, double v_max, Vector<double, 3> path_dir_vec) {
    std::array<double,3> t_pos{};
    std::array<double,3> sign{};
    //----------------------------------------------getting direction of the path for x,y,z ----------------------------
    for (int i = 0; i < 3; ++i) {
        if(path_dir_vec[i] < 0){
            sign[i] = -1;
        }else{
            sign[i] = 1;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    if (t < 0) {
        return t_pos;
    } else if (t <= t_c ) {
        for (int i = 0; i < 3; ++i) {
            t_pos[i] = start_pos[i] + 0.5 * a_max * pow(t, 2) * path_dir_vec[i];
        }
        // t_pos = start_pos + 0.5 * a_max * pow(t, 2);
    } else if((t_c < t) && (t <= (t_f - t_c) ) ) {
        for (int i = 0; i < 3; ++i) {
            t_pos[i] = start_pos[i] + a_max * t_c * (t - (t_c / 2) ) * path_dir_vec[i];
        }
       // t_pos = start_pos + a_max * t_c * (t - (t_c / 2) );
    } else if(((t_f - t_c) <= t ) && (t <= t_f) ){
        for (int i = 0; i < 3; ++i) {
            t_pos[i] = end_pos[i] - 0.5 * a_max * pow(t_f - t, 2) * path_dir_vec[i];

           // joint_t_pos = joint_endpos - 0.5 * a_max * pow(joint_t_f - t, 2) * ( joint_distance/abs(joint_distance) );
        }
     //   t_pos = end_pos - 0.5 * a_max * pow(t_f - t, 2);
    }
    return t_pos;
}

int Lin::add_middle_cfg(Configuration* config1, Configuration* config2,
                           std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps, int num_iterations2) {
    if (num_iterations2 <= 10){
        num_iterations2 = num_iterations2 + 1;
        std::vector<double> distances_joints_vec;
        distances_joints_vec.push_back(config2->get_configuration().operator[](0) * 180/M_PII - config1->get_configuration().operator[](0) * 180/M_PII);
        distances_joints_vec.push_back(config2->get_configuration().operator[](1) * 180/M_PII - config1->get_configuration().operator[](1) * 180/M_PII);
        distances_joints_vec.push_back(config2->get_configuration().operator[](2) * 180/M_PII - config1->get_configuration().operator[](2) * 180/M_PII);
        distances_joints_vec.push_back(config2->get_configuration().operator[](3) * 180/M_PII - config1->get_configuration().operator[](3) * 180/M_PII);
        distances_joints_vec.push_back(config2->get_configuration().operator[](4) * 180/M_PII - config1->get_configuration().operator[](4) * 180/M_PII);
        distances_joints_vec.push_back(config2->get_configuration().operator[](5) * 180/M_PII - config1->get_configuration().operator[](5) * 180/M_PII);

        // create new config in the middle of the 2 configs
        std::vector<double> new_cfg_vec;
        new_cfg_vec.reserve(6);
        for (int i = 0; i < 6; ++i) {
            new_cfg_vec.push_back((config1->get_configuration().operator[](i) * 180/M_PII + distances_joints_vec.at(i) * 0.5) * M_PII/180);
        }

        Configuration* cfg_middle = new Configuration(
                {new_cfg_vec.at(0), new_cfg_vec.at(1), new_cfg_vec.at(2),
                 new_cfg_vec.at(3), new_cfg_vec.at(4), new_cfg_vec.at(5)});

        // check if we exceed the limits from going from config1 to the new config
        bool second_in_limit = false;
        bool first_in_limit = checkconfiglimits(config1, cfg_middle, velocities_vec, a_max, joint_v_max, timesteps, true);
        // check if we exceed the limits from going from the new config to config 2. Here we need to use the velocities at our new config
        // so the last one that got added into the global vector (velocities are added inside the checkconfiglimits function)
        if (first_in_limit){
            // new config can be pushed into global vector
            num_iterations2 = 0;
            config_vec.push_back(cfg_middle);
            second_in_limit = checkconfiglimits(cfg_middle, config2, &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max, joint_v_max, timesteps, true);
        }else if (num_iterations2 <= 10){
            // first_in_limit is false, so we need to add another cfg between config1 and the new config
            num_iterations2 = add_middle_cfg(config1, cfg_middle, &joint_velocities_vec.at(joint_velocities_vec.size() - 1), a_max, joint_v_max, timesteps, num_iterations2);

            // after adding another cfg we still need to check if second part is in range
            second_in_limit = checkconfiglimits(cfg_middle, config2, &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max, joint_v_max, timesteps, true);
        }
        if (second_in_limit){
            // we can push config 2 also into the global vector
            num_iterations2 = 0;
            config_vec.push_back(config2);
        }else if(num_iterations2 <= 10){
            // we need to create an additional cfg between cfg_middle and config2
            num_iterations2 = add_middle_cfg(cfg_middle, config2, &joint_velocities_vec.at(joint_velocities_vec.size() - 1), a_max, joint_v_max, timesteps, num_iterations2);
        }
    }
    return num_iterations2;
}


//void Lin::add_middle_cfg2(Configuration* config1, Configuration* config2,
//                             std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps) {
//    // create vector to store configs locally
//    std::vector<Configuration*> config_temp_result;
//    config_temp_result.push_back(config1);
//    config_temp_result.push_back(config2);
//    bool first_in_limit = false;
//    bool second_in_limit = false;
//    bool rightside_inlimit = false;
//    Configuration *cfg_middle;
//    int count = 0;
//    // need to add special case of overshooting in while statement
//    while (!first_in_limit && !second_in_limit) {
//        count++;
//        std::vector<double> distances_joints_vec;
//        if (!first_in_limit) {
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - 1)->get_configuration().operator[](0) * 180 / M_PII -
//                    config_temp_result.at(0)->get_configuration().operator[](0) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - 1)->get_configuration().operator[](1) * 180 / M_PII -
//                    config_temp_result.at(0)->get_configuration().operator[](1) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - 1)->get_configuration().operator[](2) * 180 / M_PII -
//                    config_temp_result.at(0)->get_configuration().operator[](2) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - 1)->get_configuration().operator[](3) * 180 / M_PII -
//                    config_temp_result.at(0)->get_configuration().operator[](3) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - 1)->get_configuration().operator[](4) * 180 / M_PII -
//                    config_temp_result.at(0)->get_configuration().operator[](4) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - 1)->get_configuration().operator[](5) * 180 / M_PII -
//                    config_temp_result.at(0)->get_configuration().operator[](5) * 180 / M_PII);
//        }else if(!second_in_limit){
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - count)->get_configuration().operator[](0) * 180 / M_PII -
//                    config_temp_result.at(config_temp_result.size() - count - 1)->get_configuration().operator[](0) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - count)->get_configuration().operator[](1) * 180 / M_PII -
//                    config_temp_result.at(config_temp_result.size() - count - 1)->get_configuration().operator[](1) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - count)->get_configuration().operator[](2) * 180 / M_PII -
//                    config_temp_result.at(config_temp_result.size()- count - 1)->get_configuration().operator[](2) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - count)->get_configuration().operator[](3) * 180 / M_PII -
//                    config_temp_result.at(config_temp_result.size()- count - 1)->get_configuration().operator[](3) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - count)->get_configuration().operator[](4) * 180 / M_PII -
//                    config_temp_result.at(config_temp_result.size()- count - 1)->get_configuration().operator[](4) * 180 / M_PII);
//            distances_joints_vec.push_back(
//                    config_temp_result.at(config_temp_result.size() - count)->get_configuration().operator[](5) * 180 / M_PII -
//                    config_temp_result.at(config_temp_result.size()- count - 1)->get_configuration().operator[](5) * 180 / M_PII);
//        }
//
//        // create new config in the middle of the 2 configs
//        std::vector<double> new_cfg_vec;
//        new_cfg_vec.reserve(6);
//        for (int i = 0; i < 6; ++i) {
//            new_cfg_vec.push_back((config1->get_configuration().operator[](i) * 180 / M_PII + distances_joints_vec.at(i) * 0.5) * M_PII / 180);
//        }
//
//        cfg_middle = new Configuration(
//                {new_cfg_vec.at(0), new_cfg_vec.at(1), new_cfg_vec.at(2),
//                 new_cfg_vec.at(3), new_cfg_vec.at(4), new_cfg_vec.at(5)});
//
//        // check if we exceed the limits from going from config1 to the new config
//        first_in_limit = checkconfiglimits(config_vec.at(config_vec.size()-1), cfg_middle, &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max, joint_v_max, timesteps);
//        config_temp_result.push_back(cfg_middle);
//        if (first_in_limit){
//            // push latest middle point into global vector
//            config_vec.push_back(config_temp_result.at(config_temp_result.size()-1));
//            for (unsigned int i = 0; i < config_temp_result.size() - 2; ++i) {
//                // check if the now latest added cfg going to the previously iterated middle cfg does not exceed our limits
//                // this also adds the velocities of the previously iterated cfg into the global vector
//                second_in_limit = checkconfiglimits(config_vec.at(config_vec.size()-1), config_temp_result.at(config_temp_result.size() - 2 - i),
//                                                    &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max, joint_v_max, timesteps);
//                if (second_in_limit) {
//                    // both sides of the middle cfg are not outside our limits
//                    config_vec.push_back(config_temp_result.at(config_temp_result.size() - 2 - i));
//                }
//            }
//           // second_in_limit = checkconfiglimits(cfg_middle, config_temp_result.at(config_temp_result.size() - count), &joint_velocities_vec.at(joint_velocities_vec.size()-1), a_max, joint_v_max, timesteps);
//            if (second_in_limit){
//                rightside_inlimit = true;
//                // both sides of the middle cfg are not outside our limits
//                config_vec.push_back(config_temp_result.at(config_temp_result.size() - count));
//            }else{
//                rightside_inlimit = false;
//            }
//        }
//
//    }
//
//}

double Lin::calc_config_difference(Configuration* config1, Configuration* config2){

    double distance_joint1 = config1->get_configuration().operator[](0) * 180/M_PII - config2->get_configuration().operator[](0) * 180/M_PII;
    double distance_joint2 = config1->get_configuration().operator[](1) * 180/M_PII - config2->get_configuration().operator[](1) * 180/M_PII;
    double distance_joint3 = config1->get_configuration().operator[](2) * 180/M_PII - config2->get_configuration().operator[](2) * 180/M_PII;
    double distance_joint4 = config1->get_configuration().operator[](3) * 180/M_PII - config2->get_configuration().operator[](3) * 180/M_PII;
    double distance_joint5 = config1->get_configuration().operator[](4) * 180/M_PII - config2->get_configuration().operator[](4) * 180/M_PII;
    double distance_joint6 = config1->get_configuration().operator[](5) * 180/M_PII - config2->get_configuration().operator[](5) * 180/M_PII;

    return abs(distance_joint1) + abs(distance_joint2) + abs(distance_joint3) + abs(distance_joint4) + abs(distance_joint5) + abs(distance_joint6);
}

std::vector<double> Lin::calc_config_difference_to_vec(Configuration* config1, Configuration* config2){

    double distance_joint1 = config2->get_configuration().operator[](0) - config1->get_configuration().operator[](0);
    double distance_joint2 = config2->get_configuration().operator[](1) - config1->get_configuration().operator[](1);
    double distance_joint3 = config2->get_configuration().operator[](2) - config1->get_configuration().operator[](2);
    double distance_joint4 = config2->get_configuration().operator[](3) - config1->get_configuration().operator[](3);
    double distance_joint5 = config2->get_configuration().operator[](4) - config1->get_configuration().operator[](4);
    double distance_joint6 = config2->get_configuration().operator[](5) - config1->get_configuration().operator[](5);
    std::vector<double> distances_joints_vec;
    distances_joints_vec.push_back(distance_joint1);
    distances_joints_vec.push_back(distance_joint2);
    distances_joints_vec.push_back(distance_joint3);
    distances_joints_vec.push_back(distance_joint4);
    distances_joints_vec.push_back(distance_joint5);
    distances_joints_vec.push_back(distance_joint6);


    return distances_joints_vec;
}

vector<Configuration*>* Lin::check_joint1_specialcase(vector<Configuration *> *temp_vec) {
    vector<Configuration*>* result;
    // add all input configs to output vector
    result = temp_vec;

    for (int i = 0; i < temp_vec->size(); ++i) {
        double theta1 = temp_vec->at(i)->get_configuration().operator[](0) * 180/M_PII;
        if (theta1 <= 180 && theta1 >= 175){
            double new_theta1 = theta1 - 360;
            Configuration* new_cfg = new Configuration({new_theta1 * M_PII/180,
                                                        temp_vec->at(i)->get_configuration().operator[](1),
                                                        temp_vec->at(i)->get_configuration().operator[](2),
                                                        temp_vec->at(i)->get_configuration().operator[](3),
                                                        temp_vec->at(i)->get_configuration().operator[](4),
                                                        temp_vec->at(i)->get_configuration().operator[](5)});
            result->push_back(new_cfg);

        }else if (theta1 >= -180 && theta1 <= -175){
            double new_theta1 = theta1 + 360;
            Configuration* new_cfg = new Configuration({new_theta1 * M_PII/180,
                                                        temp_vec->at(i)->get_configuration().operator[](1),
                                                        temp_vec->at(i)->get_configuration().operator[](2),
                                                        temp_vec->at(i)->get_configuration().operator[](3),
                                                        temp_vec->at(i)->get_configuration().operator[](4),
                                                        temp_vec->at(i)->get_configuration().operator[](5)});
            result->push_back(new_cfg);
        }
    }
    return result;
}

bool Lin::checkconfiglimits(Configuration* config1, Configuration* config2,
                            std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps, bool add_v_to_vec) {
    std::vector<double> distances;
    distances.reserve(6);
    for (int i = 0; i < 6; ++i) {
        distances.push_back(config2->get_configuration().operator[](i) * 180/M_PII - config1->get_configuration().operator[](i) * 180/M_PII);
    }

    //calculate needed acceleration of each joint to arrive at next config in time interval given by timesteps
    double t = 1 / timesteps;
    std::vector<double> accelerations;
    accelerations.reserve(6);
    for (int i = 0; i < 6; ++i) {
        accelerations.push_back(2 * (distances.at(i) - (velocities_vec->at(i) * t)) / (t * t));
    }
    //calculate velocity of each joint after the given time interval
    std::vector<double> velocity_curr_vec;
    for (int i = 0; i < 6; ++i) {
        velocity_curr_vec.push_back( velocities_vec->at(i) + accelerations.at(i) * t );
    }

    // check if accelerations are over the limit
    for (int i = 0; i < 6; ++i) {
        bool temp = false;
        if (abs(accelerations.at(i)) > a_max){
            temp = true;
            if ( (distances.at(i) > 0 && accelerations.at(i) < 0) || (distances.at(i) < 0 && accelerations.at(i) > 0)){
                this->movement_case = 2;
                std::cout << "Movement Special Case = " << movement_case << ". Joint cannot decelerate fast enough." << std::endl;
                std::cout << "Therefore we don't create a config for this point." << std::endl;
            }
        }
        if(temp){
            return false;
        }
    }


    // check if velocities are over the limit
    for (int i = 0; i < 6; ++i) {
        if (abs(velocities_vec->at(i)) > joint_v_max[i]){
            return false;
        }
    }
    //push current velocity in global vector so we store all velocities at the points
    if (add_v_to_vec){
        joint_velocities_vec.push_back(velocity_curr_vec);
    }

    return true;
}
