/*--------------------------------------------------
 * File: shortpath.cpp
 *
 * Usage: No input
 *        ./shortpath
 *
 * Output: writes to terminal and file shortpath.log
 *
 * Purpose: Dijkstra Shortest Path Algorithm demonstration
 *
 * Reference:
 *  https://www.coursera.org/learn/c-plus-plus-a/lecture/nkelp/2-5-dijkstra-shortest-path
 *
 * Author: Don Cohoon - October 2020
 *--------------------------------------------------
 */
#include <iostream>
#include <set>
#include <iterator>
#include <cassert>
#include <fstream>

using namespace std;

enum Ok { yes, no, maybe };
enum Build { cont, finished, nomore };
// TODO: Adjust MAX values based on map
const double MAX_WEIGHT = 1000;
const double MAX_EDGES  = 100;
// TODO: move START/FINISH as arguments to main
//       or map file
const double START  = 1;
const double FINISH = 9;
ofstream logfile;

//--------------------------------------------------
class Edge
{
  public:
     Edge () { _s=0;_d=0;_w=0;_n=""; }
     Edge (double source,
           double dest,
           double weight,
           string name ) { _s = source; 
                           _d = dest;
                           _w = weight;
                           _n = name; }
     const double get_source () const { return _s;}
     const double get_dest   () const { return _d;}
     const double get_weight () const { return _w;}
     const string get_name   () const { return _n;}
     void         set_weight (const double w) { _w = w;}

     friend ostream& operator << (ostream& os, const Edge& e)
     {
         os << "(" << e._s << "," << e._d << "):" << e._w;
         return os ;
     }
     friend bool operator < ( const Edge& left, const Edge& right)
     { // IMPORTANT: This is the sort/duplicate remover
       //             want to sort by weight
         return left._w+(left._s * MAX_EDGES) < right._w+(right._s * MAX_EDGES);
     }
     friend bool operator > ( const Edge& left, const Edge& right)
     {
         return left._w > right._w;
     };
     friend bool operator == ( const Edge& left, const Edge& right)
     {
         return ( left._s == right._s &&
                  left._d == right._d &&
                  left._w == right._w);
     }

  private:
     double _s, _d, _w;
     string _n;
}; // Edge

//--------------------------------------------------
//--------------------------------------------------
// Populate given Edge set
Ok insert_edge( set<Edge> &e,
                double    source, 
                double    dest, 
                double    weight,
                string    name )
{ 
  auto rslt = e.insert(Edge(source, dest, weight, name));
  if ( rslt.second && rslt.first != e.end() )
  {
    logfile << "INFO: " << static_cast<string>((*rslt.first).get_name())
            << " " << *rslt.first
            << " inserted" << endl;
    return yes;
  }
  else
  {
    cout << "ERROR: " << static_cast<string>((*rslt.first).get_name())
         << " " << *rslt.first
         << " prevented insertion" << endl;
    return no;
  }
} // insert_edge

//--------------------------------------------------
/*
 *    Build ALL road map, with weights
 *
 *    TODO: Read from map file
 */
void build_map( set<Edge> &m )
{ // https://www.coursera.org/learn/c-plus-plus-a/lecture/nkelp/2-5-dijkstra-shortest-path
  Ok ok;
  //  S=1
  //                     Source Dest Weight  Name
  ok = insert_edge(m,1,     2,   4,     "Start"); assert( ok == yes); 
  ok = insert_edge(m,1,     5,   7,     "Start"); assert( ok == yes);
  ok = insert_edge(m,1,     3,   3,     "Start"); assert( ok == yes);
  //  A=2
  ok = insert_edge(m,2,     4,   1,     "A"); assert( ok == yes);
  //  B=3
  ok = insert_edge(m,3,     1,   3,     "B"); assert( ok == yes);
  ok = insert_edge(m,3,     5,   4,     "B"); assert( ok == yes);
  //  C=4
  ok = insert_edge(m,4,     6,   1,     "C"); assert( ok == yes);
  ok = insert_edge(m,4,     5,   3,     "C"); assert( ok == yes);
  //  D=5
  ok = insert_edge(m,5,     7,   5,     "D"); assert( ok == yes);
  ok = insert_edge(m,5,     9,   3,     "D"); assert( ok == yes);
  ok = insert_edge(m,5,     6,   1,     "D"); assert( ok == yes);
  //  E=6
  ok = insert_edge(m,6,     8,   2,     "E"); assert( ok == yes);
  ok = insert_edge(m,6,     9,   4,     "E"); assert( ok == yes);
  //  F=7
  //  G=8
  ok = insert_edge(m,8,     6,   2,     "G"); assert( ok == yes);
  ok = insert_edge(m,8,     9,   3,     "G"); assert( ok == yes);
  //  T=9
  ok = insert_edge(m,9,     9,   0,     "Finish"); assert( ok == yes);
} // build_map

//--------------------------------------------------
// Print set
void print_set( ofstream &ofs,
                string header,
                set<Edge> &e,
                double cost = 0)
{
  ofs << "===> " << header << endl;
  if (cost>0) ofs << "Total cost: " << cost << endl;
  for (auto it=e.begin(); it != e.end(); ++it)     
    ofs //<< *it
           << "[S:" << (*it).get_source() << ","
           <<  "D:" << (*it).get_dest()   << "]"
           << ":W:" << (*it).get_weight() << " "
           << (*it).get_name()
           << endl;
  ofs << "-----------" << endl;
}

//--------------------------------------------------
// Print set
void print_set( ostream &os,
                string header,
                set<Edge> &e,
                double cost = 0)
{
  os << "===> " << header << endl;
  if (cost>0) os << "Total cost: " << cost << endl;
  for (auto it=e.begin(); it != e.end(); ++it)     
    os //<< *it
             << "[S:" << (*it).get_source() << ","
             <<  "D:" << (*it).get_dest()   << "]"
             << ":W:" << (*it).get_weight() << " "
             << (*it).get_name()
             << endl;
  os << "-----------" << endl;
} // print_set

//--------------------------------------------------
// => Get <all> successors from <starting> point
//    if: Not in <ignore> set; or edge to FINISH
//    put into <found> set (what's reachable)
Ok get_successors ( set<Edge> all,
                    set<Edge> &found,
                    set<Edge> ignore,
                    Edge      starting)
{
  Ok ok;
  auto it = all.begin();
  while(it != all.end())
  {
    if ( (*it).get_source() == starting.get_dest()  &&
         ignore.find(*it)   == ignore.end() &&
         (*it).get_dest()   == FINISH  )
    {
         ok = insert_edge(found,(*it).get_source(),
                               (*it).get_dest(),
                               (*it).get_weight(),
                               (*it).get_name());
         assert( ok == yes); 
         break; // dest reached
    }
    else if ( (*it).get_source() == starting.get_dest()  &&
               ignore.find(*it)  == ignore.end()  )
    {
         ok = insert_edge(found,(*it).get_source(),
                               (*it).get_dest(),
                               (*it).get_weight(),
                               (*it).get_name());
         assert( ok == yes); 
    }
    it++;
  }

  return yes;
} // get_successors

//--------------------------------------------------
// => Get lowest weight successor from starting point
Edge get_lowest_successor ( set<Edge> es,
                            Edge      starting,
                            double    best_weight )
{
  Edge lowest_weight;
  double current_weight=best_weight;
  auto it = es.begin();
  while(it != es.end())
  {
    if (
         (
           (*it).get_source() == starting.get_dest() &&
           (*it).get_weight() <  current_weight && 
           (*it).get_dest()   != START // TODO: change to -> back where I came from
         ) ||
         (
           (*it).get_dest() == FINISH
         )
       )
      {
         lowest_weight = (*it); 
         current_weight = lowest_weight.get_weight();
         logfile << "Lowest weight: " << (*it) << " from " << starting.get_name() << endl;
      }
    it++;
  }

  return lowest_weight;
} // get_successors

//--------------------------------------------------
// => Get sum of weights from Edge set
double get_sum_weights ( set<Edge> es )
{
  double weights=0;
  auto it = es.begin();
  while(it != es.end())
  {
    weights =  weights + (*it).get_weight();
    it++;
  } 

  return weights;
} // get_sum_weights

//--------------------------------------------------
// => Add Edge to best and closed, remove from open
Ok add_edge_to_path ( Edge theEdge,
                      set<Edge> &open,
                      set<Edge> &closed,
                      set<Edge> &best)
{
  auto count = open.erase(theEdge);
  if ( count == 1 ) // should be one item erased
  {
    logfile << "INFO: " << theEdge.get_name() << " erased from open" << endl;
  }
  else
  {
    logfile << "ERROR: " << theEdge.get_name() << " NOT erased from open" << endl;
    return no;
  }
  //
  auto rslt = closed.insert(theEdge);
  if ( rslt.second && rslt.first != closed.end() )
  {
    logfile << "INFO: " << static_cast<string>((*rslt.first).get_name())
            << " " << *rslt.first
            << " inserted" << endl;
  }
  else
  {
    cout << "ERROR: " << static_cast<string>((*rslt.first).get_name())
         << " " << *rslt.first
         << " prevented insertion" << endl;
    return no;
  }
  //
  rslt = best.insert(theEdge); 
  if ( rslt.second && rslt.first != best.end() )
  {
    logfile << "INFO: " << static_cast<string>((*rslt.first).get_name())
            << " " << *rslt.first
            << " inserted" << endl;
  }
  else
  {
    cout << "ERROR: " << static_cast<string>((*rslt.first).get_name())
         << " " << *rslt.first
         << " prevented insertion" << endl;
    return no;
  }
  //

  return yes;
} // add_edge

//--------------------------------------------------
// Build up shortest path(best) from current_node
// => return cont, finished
Build build_path ( Edge     &current_node,
                   set<Edge> paths,
                   set<Edge> &open,
                   set<Edge> &closed,
                   set<Edge> &best,
                   double    &current_weight,
                   Edge      &lowest_weight,
                   double    &current_cost,
                   double    &total_cost)
{
  Ok ok;
  print_set(logfile, "Best Set", best);

  // => Put all immediate sucessors of current_node, into open set (what's reachable) if not in closed set
  ok = get_successors ( paths, open, closed, current_node ); assert( ok == yes);
  print_set(logfile,"Open Set", open);

  // => Pick "open node" of "open set" whose "cost" is "Least"
  current_weight=MAX_WEIGHT;
  lowest_weight = get_lowest_successor ( open, current_node, current_weight);
  current_weight = lowest_weight.get_weight();

  // => No more successors, failed "no path"
  if ( current_weight == 0 )
  {
    logfile << "Failed Path" << endl;
    return finished;
  }

  // => If reached dest, stop
  if ( lowest_weight.get_dest() == FINISH )
  {
    logfile << "dest: " << lowest_weight.get_dest() << " FINISHed" << endl;
    total_cost = total_cost + lowest_weight.get_weight();
    // Add lowest_weight Edge to best and closed, remove from open
    ok = add_edge_to_path ( lowest_weight, open, closed, best); assert( ok == yes);
    return finished;
  }

  // Cnj = Cost of Edge n,j; C(S->n) = current cost, back to S; C(S->n) + Cnj = Total Cost
  current_cost = get_sum_weights ( best );
  total_cost   = current_cost + lowest_weight.get_weight();
  current_node = lowest_weight;

  // Add lowest_weight Edge to best and closed, remove from open
  ok = add_edge_to_path ( lowest_weight, open, closed, best); assert( ok == yes);

  // Update logfile
  logfile << "-----------" << endl;
  logfile << "Moved " << lowest_weight  << " to closed and best sets" << endl;
  logfile << "Current Cost:" << current_cost << endl;
  logfile << "Total Cost:" << total_cost << " of adding edge " <<  lowest_weight << endl;
  logfile << "-----------" << endl;

  // If not FINISHed, repeat path search
  if ( lowest_weight.get_dest() != FINISH )
  {
    logfile << "Not yet finished, current_node: " << current_node
            << " repeat path search" << endl;
    return cont;
  }
  return finished;
} // build_path

//--------------------------------------------------
int main() {
  Ok ok;
  Build bp;
  set<Edge> paths;
  set<Edge> closed;
  set<Edge> open;
  set<Edge> best;
  Edge current_node;
  Edge lowest_weight;
  double current_weight=MAX_WEIGHT;
  double current_cost=0;
  double total_cost=0;

  logfile.open ("shortpath.log",ios_base::app);
  // Build initial map
  build_map( paths );

  // => Put [S]tart in "closed set" (known shortest path), weight=0
  //                      Source Dest   Weight  Name
  ok = insert_edge(closed,START, START, 0,     "Start"); assert( ok == yes); 
  ok = insert_edge(best,  START, START, 0,     "Start"); assert( ok == yes); 
  current_node = Edge(    START, START, 0,     "Start");

  do
  {
    do
    { // Build new path from current_node
      bp = build_path ( current_node, paths, open, closed, best,
                        current_weight, lowest_weight, current_cost, total_cost);
    } while ( bp == cont ); 

    if ( total_cost > 0 )
    {  // We have a good path
       print_set( cout, "Path", best, total_cost);
    }

    if ( open.size() == 0 ) // Tried all roads from Start
      bp = nomore;

    // Check for another path to Finish, starting back at Start
    open.clear();
    best.clear();
    ok = insert_edge(best, START,START,0,"Start"); assert( ok == yes); 
    current_node = Edge(   START,START,0,"Start");
    total_cost = current_cost = 0;
  } while ( bp != nomore );

  logfile.close();

}
