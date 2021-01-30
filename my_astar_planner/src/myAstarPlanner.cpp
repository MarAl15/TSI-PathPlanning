/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
/********************************************************************
* Modified by Mar Alguacil
*********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

// para debugging
#include <sstream>
#include <string>

# define M_PI 3.14159265358979323846





//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

    list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);
    bool isContains(list<coupleOfCells> & list1, int cellID);


    MyastarPlanner::MyastarPlanner()
    : costmap_ros_(NULL), initialized_(false){}

    MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false){
        initialize(name, costmap_ros);
    }



    void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);

            //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            //world_model_ = new base_local_planner::CostmapModel(*costmap_);
            //////////////// MAR /////////////////
            world_model_ = NULL;
            footprint = costmap_ros_->getRobotFootprint();
            ////////////// FIN MAR ///////////////

            //el plan se va a publicar en el topic "planTotal"
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);
            //los puntos del espacio de búsqueda se visualizan en "visualization_marker"
            marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
            marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
            //////////////// MAR /////////////////
            marker_Path_publisher = private_nh.advertise<visualization_msgs::Marker>("path_list", 1000);
            ////////////// FIN MAR ///////////////
            marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);


            //////////////// MAR /////////////////
            N = costmap_->getSizeInCellsX();
            M = costmap_->getSizeInCellsY();
            OBS = 254;
            ////////////// FIN MAR ///////////////

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    //we need to take the footprint of the robot into account when we calculate cost to obstacles
    bool MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return true;
        }

        if( world_model_==NULL )
            world_model_ = new base_local_planner::CostmapModel(*costmap_);

        if(footprint.size()==0)
            footprint = costmap_ros_->getRobotFootprint();

        //if we have no footprint... do nothing
        if(footprint.size() < 3)
            return true;

        //check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
        return ((footprint_cost<0) ? true : ((footprint_cost>=OBS) ? true : false));
    }

    // Devuelve true si la probabilidad de colisión es alta o (x_i, y_i) se encuentra fuera del mapa, false en caso contrario
    bool MyastarPlanner::collision(unsigned int x_i, unsigned int y_i){
        double wx, wy;
        costmap_->mapToWorld( x_i, y_i, wx, wy);
        if( (x_i>=0) && (x_i<N) && (y_i>=0) && (y_i<M) ){
            return (footprintCost(wx, wy, 0) &&
                    footprintCost(wx, wy, M_PI/2) &&
                    footprintCost(wx, wy, M_PI) &&
                    footprintCost(wx, wy, 3*M_PI/2) );
        }
        else
            return true;
    }

    bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

        //***********************************************************
        // Inicio de gestion de ROS
        //***********************************************************
        if(!initialized_){
            ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        closedList.clear();
        openList.clear();

        //obtenemos el costmap global  que está publicado por move_base.
        costmap_ = costmap_ros_->getCostmap();


        //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
        //esto es importante para evitar errores de transformaciones de coordenadas.
        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                        costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        tf::Stamped<tf::Pose> goal_tf;
        tf::Stamped<tf::Pose> start_tf;

        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        //obtenemos la orientación start y goal en start_yaw y goal_yaw.
        double useless_pitch, useless_roll, goal_yaw, start_yaw;
        start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


        /**************************************************************************/
        /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
        /****************************************************************************/

        //pasamos el goal y start a estructura coupleOfCells
        coupleOfCells cpstart, cpgoal;
        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;
        unsigned int mgoal_x, mgoal_y;
        costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
        cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
        cpgoal.parent=0;
        cpgoal.gCost=0;
        cpgoal.hCost=0;
        cpgoal.fCost=0;

        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;
        unsigned int mstart_x, mstart_y;
        costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
        cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
        cpstart.parent =cpstart.index;
        cpstart.gCost = 0;
        cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);

        //insertamos la casilla inicial en abiertos
        MyastarPlanner::openList.push_back(cpstart);
        ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
        ROS_INFO("Index del goal: %d", cpgoal.index );

        //////////////// MAR /////////////////
        bool found = false;
        ////////////// FIN MAR ///////////////

        /**************************************************************************/
        /*************** GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
        /****************************************************************************/

        //visualization_msgs::Marker points;// definida en la clase como markers_OpenList
        inicializaMarkersPoints(markers_OpenList,"openList", 0,0.0f,1.0f,0.7f);
        inicializaMarkersPoints(markers_ClosedList,"closedList", 1,1.0f,0.0f,0.0f);
        inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f,1.0f);
        //////////////// MAR /////////////////
        inicializaMarkersPoints(markers_Path, "path", 3, 0.0f, 0.0f,1.0f);
        ////////////// FIN MAR ///////////////

        limpiaMarkers(marker_Open_publisher, markers_ClosedList);
        limpiaMarkers(marker_Closed_publisher, markers_OpenList);
        limpiaMarkers(marker_Path_publisher, markers_Path);




        /**************************************************************************/
        /*************** FIN GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
        /****************************************************************************/


        unsigned int explorados = 0;
        unsigned int currentIndex = cpstart.index;
        //////////////// MAR /////////////////
        coupleOfCells COfCells;

        ros::Time time_begin = ros::Time::now();
        ////////////// FIN MAR ///////////////
        while (!MyastarPlanner::openList.empty()) //while the open list is not empty continuie the search
        {


            //escoger UNA casilla DE abiertos
            COfCells= openList.front();
            openList.pop_front();
            currentIndex=COfCells.index;



            //obtenemos un iterador al elemento en la lista de abiertos
            list<coupleOfCells>::iterator it=getPositionInList(openList,currentIndex);



            //y la insertamos en cerrados
            MyastarPlanner::closedList.push_back(COfCells);


            // if the currentCell is the goalCell: success: path found
            if(currentIndex==cpgoal.index)
            {
                //////////////// MAR /////////////////
                found = true;
                break;
                ////////////// FIN MAR ///////////////
            }



            //search the neighbors of the current Cell
            vector <unsigned int> neighborCells=findFreeNeighborCell(currentIndex);

            //neighbors that exist in the closedList are ignored
            vector <unsigned int> neighborNotInClosedList;
            for(uint i=0; i<neighborCells.size(); i++)
            {
                if(!isContains(closedList,neighborCells[i]))
                {
                  neighborNotInClosedList.push_back(neighborCells[i]);
                }
            }


            //search the neighbors that already exist in the open List
            vector <unsigned int> neighborsNotInOpenList;

            //////////////// MAR /////////////////
            // Actualizo nodos de abiertos si encuentro un camino mejor
            double c, new_cost;
            list<coupleOfCells>::iterator node; // getPositionInList
            for(vector <unsigned int>::iterator it=neighborNotInClosedList.begin(); it!=neighborNotInClosedList.end(); it++)
            {
                node = getPositionInList(openList,*it);
                if( node !=openList.end() ){

                    // Actualizamos el camino si es mejor
                    c = getMoveCost(currentIndex,(*node).index);
                    new_cost = COfCells.gCost+c;
                    if( new_cost<(*node).gCost ){
                        (*node).gCost = new_cost;
                        (*node).parent = currentIndex;
                        (*node).fCost = (*node).gCost + (*node).hCost;
                    }

                }else
                    neighborsNotInOpenList.push_back(*it);
            }

            //add the neighbors that are not in the open list to the open list and mark the current cell as their parent
            addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, COfCells.gCost, cpgoal.index);
            ////////////// FIN MAR ///////////////

            explorados++;

            //PINTO ABIERTOS
            //Anyadir neighborCells a points. pushback()
            //visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
            //visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells.index);
        }
        //////////////// MAR /////////////////
        ros::Time time_end = ros::Time::now();
        double secs = time_end.toSec() - time_begin.toSec();
        ROS_INFO("Tiempo: %f", (double)secs);
        ////////////// FIN MAR ///////////////

        if(found){
            limpiaMarkers(marker_Open_publisher, markers_ClosedList);
            limpiaMarkers(marker_Closed_publisher, markers_OpenList);

            //////////////// MAR /////////////////
            bool cost = false;

            vector <unsigned int> closed;
            for(list<coupleOfCells>::iterator it=closedList.begin(); it!=closedList.end(); it++)
                closed.push_back((*it).index);

            vector <unsigned int> open;
            for(list<coupleOfCells>::iterator it=openList.begin(); it!=openList.end(); it++)
                open.push_back((*it).index);

            unsigned int length_node = 0;
            ////////////// FIN MAR ///////////////

            //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
            //vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
            ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());
            //ros::Duration(10).sleep();
            //convertimos goal a poseStamped nueva
            geometry_msgs::PoseStamped pose;
            pose.header.stamp =  ros::Time::now();
            pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
            pose.pose.position.x = goal_x;
            pose.pose.position.y = goal_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            //lo añadimos al plan
            plan.push_back(pose);
            length_node++;


            coupleOfCells currentCouple = cpgoal;
            unsigned int currentParent = COfCells.parent;
            //ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
            //ros::Duration(1).sleep();

            while (currentParent != cpstart.index) //e.d. mientras no lleguemos al nodo start
            {
                //////////////// MAR /////////////////
                visualizaCelda(marker_Path_publisher, markers_Path, currentParent);
                remove(closed.begin(), closed.end(), currentParent);
                ////////////// FIN MAR ///////////////
                //encontramos la posición de currentParent en cerrados
                list<coupleOfCells>::iterator it=getPositionInList(closedList,currentParent);

                //hacemos esa posición que sea el currentCouple
                coupleOfCells currentCouple;
                currentCouple.index=currentParent;
                currentCouple.parent=(*it).parent;
                currentCouple.gCost=(*it).gCost;
                currentCouple.hCost=(*it).hCost;
                currentCouple.fCost=(*it).fCost;

                //////////////// MAR /////////////////
                if(!cost){
                    cost = true;
                    ROS_INFO("Coste total: %f ", (double)(*it).fCost );
                }
                ////////////// FIN MAR ///////////////

                //creamos una PoseStamped con la informaciuón de currentCouple.index

                    //primero hay que convertir el currentCouple.index a world coordinates
                unsigned int mpose_x, mpose_y;
                double wpose_x, wpose_y;

                costmap_->indexToCells((*it).index, mpose_x, mpose_y);
                costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

                //ros::Duration(1).sleep();

                    //después creamos la pose
                geometry_msgs::PoseStamped pose;
                pose.header.stamp =  ros::Time::now();
                pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
                pose.pose.position.x = wpose_x;
                pose.pose.position.y = wpose_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                //insertamos la pose en el plan
                plan.push_back(pose);
                //hacemos que currentParent sea el parent de currentCouple
                currentParent = (*it).parent;

                length_node++;
            }

            ROS_INFO("Sale del bucle de generacion del plan.");
            std::reverse(plan.begin(),plan.end());

            //////////////// MAR /////////////////
            ROS_INFO("Longitud del camino (nodos): %u ", length_node );

            visualizaLista(marker_Open_publisher, markers_OpenList, open);
            visualizaLista(marker_Closed_publisher, markers_ClosedList, closed);
            ////////////// FIN MAR ///////////////


            //lo publica en el topic "planTotal"
            //publishPlan(plan);
            return true;
        }
        if(openList.empty())  // if the openList is empty: then failure to find a path
        {
            ROS_INFO("Failure to find a path !");
            return false;
            // exit(1);
        }

    };


    //calculamos H como la distancia euclídea hasta el goal
    double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
        unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
        double wstart_x, wstart_y, wgoal_x, wgoal_y;

        //trasformamos el indice de celdas a coordenadas del mundo.
        //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

        costmap_->indexToCells(start, mstart_x, mstart_y);
        costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
        costmap_->indexToCells(goal, mgoal_x, mgoal_y);
        costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

        return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));
    }


    //comparamos F para dos nodos.
    bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2)
    {
        return c1.fCost < c2.fCost;
    }

/*******************************************************************************/
//Function Name: getPositnionInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID)
{
    for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
        if (it->index == cellID)
            return it;
    }

    return list1.end();
}


 /*******************************************************************************
 * Function Name: findFreeNeighborCell
  * Inputs: the row and columun of the current Cell
  * Output: a vector of free neighbor cells of the current cell
  * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
  * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID){
    unsigned int mx, my;
    costmap_->indexToCells(CellID,mx,my);

    vector <unsigned int>  freeNeighborCells;

    for (int x=-1;x<=1;x++)
        for (int y=-1; y<=1;y++){
            //////////////// MAR /////////////////
            if( (!(x==0 && y==0)) && costmap_->getCost(mx+x,my+y)<127 && !collision((unsigned int)mx+x, (unsigned int)my+y) ){
                int index = costmap_->getIndex(mx+x,my+y);
                freeNeighborCells.push_back(index);
            }
            ////////////// FIN MAR ///////////////
        }
    return  freeNeighborCells;
}


/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list or in the closed list
/*********************************************************************************/
bool isContains(list<coupleOfCells> & list1, int cellID)
{
    for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
        if (it->index == cellID)
            return true;
    }

    return false;
}

double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
    //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
    return calculateHCost(here,there);
}

/*******************************************************************************/
//Function Name: addNeighborCellsToOpenList
//Inputs: the open list, the neighbors Cells and the parent Cell
//Output:
//Description: it is used to add the neighbor Cells to the open list
/*********************************************************************************/
void MyastarPlanner::addNeighborCellsToOpenList(list<coupleOfCells> & OPL, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) //,float tBreak)
{
    //vector <coupleOfCells> neighborsCellsOrdered;
    for(uint i=0; i< neighborCells.size(); i++)
    {
        coupleOfCells CP;
        CP.index = neighborCells[i]; //insert the neighbor cell
        CP.parent = parent; //insert the parent cell
        CP.gCost = gCostParent + getMoveCost(parent, CP.index);
        CP.hCost = getMoveCost(CP.index, goalCell);
        CP.fCost = CP.gCost + CP.hCost;

        OPL.push_back(CP);
    }

    //////////////// MAR /////////////////
    // Ordenamos el vector
    OPL.sort(compareFCost);
    ////////////// FIN MAR ///////////////
}


  /********VISUALIZAR ESPACIO DE BUSQUEDA *************************/

void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
        marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
        marker.header.stamp =  ros::Time::now();
        marker.ns = ns;

        marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
        marker.pose.orientation.w = 0.0;



        marker.id = id;



        marker.type = visualization_msgs::Marker::POINTS;




        // POINTS markers use x and y scale for width/height respectively
        marker.scale.x = costmap_->getResolution();
        marker.scale.y = costmap_->getResolution();


        // Points are green
        marker.color.g = g;
        marker.color.r = r;
        marker.color.b = b;
        marker.color.a = 1.0;

    }

void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
        marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
        marker.header.stamp =  ros::Time::now();
        marker.ns = ns;

        marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
        marker.pose.orientation.w = 0.0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;



        marker.id = id;



        marker.type = visualization_msgs::Marker::SPHERE;




        //Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
        marker.scale.x = marker.scale.y = 0.5;
       // marker.scale.y = costmap_->getResolution();


        // Points are green
        marker.color.g = g;
        marker.color.r = r;
        marker.color.b = b;
        marker.color.a = 1.0;

    }

void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
        //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0; //¿?

        marker.points.push_back(p); //anyado el punto inicial
        where.publish(marker); //lo publico
        //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
        }

void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z) {
        //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        where.publish(marker); //lo publico
        //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
        }

void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
        unsigned int mpose_x, mpose_y;
        double wpose_x, wpose_y;
        costmap_->indexToCells(index, mpose_x, mpose_y);
        costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
        visualizaCoords(where, marker, wpose_x, wpose_y);

    }

 void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista) {
     for(vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i)
       {
         unsigned int mpose_x, mpose_y;
         double wpose_x, wpose_y;
         costmap_->indexToCells(*i, mpose_x, mpose_y);
         costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
         //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
         geometry_msgs::Point p;
         p.x = wpose_x;
         p.y = wpose_y;
         p.z = 0; //¿?

         marker.points.push_back(p);
        }
      where.publish(marker);
 //quitar neighborCells de points .popback
 }
 void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
    if (!marker.points.empty()){
        marker.action = visualization_msgs::Marker::DELETE;
        where.publish(marker);
        marker.action = visualization_msgs::Marker::ADD;
    }
    marker.points.clear();


 }
}
