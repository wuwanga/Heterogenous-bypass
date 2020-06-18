#include "booksim.hpp"
#include <vector>
#include <sstream>

#include "kncube.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"

// UNT added bypass
#include "../cpu-sim/globals.h"

KNCube::KNCube( const Configuration &config, bool mesh, int seperate_network_ratio, int net_num ) :
Network( config )
{
   _mesh = mesh;

   _ComputeSize( config );
   _Alloc( );
   _BuildNet( config, seperate_network_ratio, net_num );
}

void KNCube::_ComputeSize( const Configuration &config )
{
   _k = config.GetInt( "k" );
   _n = config.GetInt( "n" );

   gK = _k; gN = _n;

   _size     = powi( _k, _n );
   _channels = 2*_n*_size;

   _sources = _size;
   _dests   = _size;
}

void KNCube::_BuildNet( const Configuration &config, int seperate_network_ratio, int net_num )
{
   int left_node;
   int right_node;

   int right_input;
   int left_input;

   int right_output;
   int left_output;

   ostringstream router_name;

   for ( int node = 0; node < _size; ++node ) {

      router_name << "router";

      for ( int dim_offset = _size / _k; dim_offset >= 1; dim_offset /= _k ) {
         router_name << "_" << ( node / dim_offset ) % _k;
      }

      _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
                                          node, 2*_n + 1, 2*_n + 1, seperate_network_ratio, net_num );

      router_name.seekp( 0, ios::beg );

      for ( int dim = 0; dim < _n; ++dim ) {

         left_node  = _LeftNode( node, dim );
         right_node = _RightNode( node, dim );

         //
         // Current (N)ode
         // (L)eft node
         // (R)ight node
         //
         //   L--->N<---R
         //   L<---N--->R
         //

         right_input = _LeftChannel( right_node, dim );
         left_input  = _RightChannel( left_node, dim );

         _routers[node]->AddInputChannel( &_chan[right_input], &_chan_cred[right_input] );
         _routers[node]->AddInputChannel( &_chan[left_input], &_chan_cred[left_input] );

         right_output = _RightChannel( node, dim );
         left_output  = _LeftChannel( node, dim );

         _routers[node]->AddOutputChannel( &_chan[right_output], &_chan_cred[right_output] );
         _routers[node]->AddOutputChannel( &_chan[left_output], &_chan_cred[left_output] );
      }

      _routers[node]->AddInputChannel( &_inject[node], &_inject_cred[node] );
      _routers[node]->AddOutputChannel( &_eject[node], &_eject_cred[node] );
   }
}

int KNCube::_LeftChannel( int node, int dim )
{
   // The base channel for a node is 2*_n*node
   int base = 2*_n*node;

   // The offset for a left channel is 2*dim + 1
   int off  = 2*dim + 1;

   return( base + off );
}

int KNCube::_RightChannel( int node, int dim )
{
   // The base channel for a node is 2*_n*node
   int base = 2*_n*node;

   // The offset for a right channel is 2*dim 
   int off  = 2*dim;

   return( base + off );
}

int KNCube::_LeftNode( int node, int dim )
{
   int k_to_dim = powi( _k, dim );
   int loc_in_dim = ( node / k_to_dim ) % _k;
   int left_node;

   // if at the left edge of the dimension, wraparound
/*
   if ( loc_in_dim == 0 ) {
      left_node = node + (_k-1)*k_to_dim;
   } else {
      left_node = node - k_to_dim;
   }
*/
   if ( dim == 0 ) {
      if ( node == 0 ) {
         left_node = 3;
      } else if ( node == 1 ) {
         left_node = 0;
      } else if ( node == 2 ) {
         left_node = 5;
      } else if ( node == 3 ) {
         left_node = 1;
      } else if ( node == 4 ) {
         left_node = 2;
      } else if ( node == 5 ) {
         left_node = 4;
      } else if ( node == 6 ) {
         left_node = 10;
      } else if ( node == 7 ) {
         left_node = 11;
      } else if ( node == 8 ) {
         left_node = 6;
      } else if ( node == 9 ) {
         left_node = 7;
      } else if ( node == 10 ) {
         left_node = 8;
      } else if ( node == 11 ) {
         left_node = 9;
      } else if ( node == 12 ) {
         left_node = 16;
      } else if ( node == 13 ) {
         left_node = 17;
      } else if ( node == 14 ) {
         left_node = 12;
      } else if ( node == 15 ) {
         left_node = 13;
      } else if ( node == 16 ) {
         left_node = 14;
      } else if ( node == 17 ) {
         left_node = 15;
      } else if ( node == 18 ) {
         left_node = 23;
      } else if ( node == 19 ) {
         left_node = 18;
      } else if ( node == 20 ) {
         left_node = 19;
      } else if ( node == 21 ) {
         left_node = 20;
      } else if ( node == 22 ) {
         left_node = 21;
      } else if ( node == 23 ) {
         left_node = 22;
      } else if ( node == 24 ) {
         left_node = 27;
      } else if ( node == 25 ) {
         left_node = 24;
      } else if ( node == 26 ) {
         left_node = 29;
      } else if ( node == 27 ) {
         left_node = 25;
      } else if ( node == 28 ) {
         left_node = 26;
      } else if ( node == 29 ) {
         left_node = 28;
      } else if ( node == 30 ) {
         left_node = 34;
      } else if ( node == 31 ) {
         left_node = 35;
      } else if ( node == 32 ) {
         left_node = 30;
      } else if ( node == 33 ) {
         left_node = 31;
      } else if ( node == 34 ) {
         left_node = 32;
      } else if ( node == 35 ) {
         left_node = 33;
      }
   } else if ( dim == 1 ) {
      if ( node == 0 ) {
         left_node = 30;
      } else if ( node == 1 ) {
         left_node = 25;
      } else if ( node == 2 ) {
         left_node = 26;
      } else if ( node == 3 ) {
         left_node = 27;
      } else if ( node == 4 ) {
         left_node = 28;
      } else if ( node == 5 ) {
         left_node = 35;
      } else if ( node == 6 ) {
         left_node = 0;
      } else if ( node == 7 ) {
         left_node = 31;
      } else if ( node == 8 ) {
         left_node = 32;
      } else if ( node == 9 ) {
         left_node = 33;
      } else if ( node == 10 ) {
         left_node = 34;
      } else if ( node == 11 ) {
         left_node = 5;
      } else if ( node == 12 ) {
         left_node = 6;
      } else if ( node == 13 ) {
         left_node = 7;
      } else if ( node == 14 ) {
         left_node = 8;
      } else if ( node == 15 ) {
         left_node = 9;
      } else if ( node == 16 ) {
         left_node = 10;
      } else if ( node == 17 ) {
         left_node = 11;
      } else if ( node == 18 ) {
         left_node = 12;
      } else if ( node == 19 ) {
         left_node = 1;
      } else if ( node == 20 ) {
         left_node = 2;
      } else if ( node == 21 ) {
         left_node = 3;
      } else if ( node == 22 ) {
         left_node = 4;
      } else if ( node == 23 ) {
         left_node = 17;
      } else if ( node == 24 ) {
         left_node = 18;
      } else if ( node == 25 ) {
         left_node = 19;
      } else if ( node == 26 ) {
         left_node = 20;
      } else if ( node == 27 ) {
         left_node = 21;
      } else if ( node == 28 ) {
         left_node = 22;
      } else if ( node == 29 ) {
         left_node = 23;
      } else if ( node == 30 ) {
         left_node = 24;
      } else if ( node == 31 ) {
         left_node = 13;
      } else if ( node == 32 ) {
         left_node = 14;
      } else if ( node == 33 ) {
         left_node = 15;
      } else if ( node == 34 ) {
         left_node = 16;
      } else if ( node == 35 ) {
         left_node = 29;
      }
   }

//   printf("------dim:%d, node:%d, left_node:%d\n", dim, node, left_node);


   return left_node;
}

int KNCube::_RightNode( int node, int dim )
{
   int k_to_dim = powi( _k, dim );
   int loc_in_dim = ( node / k_to_dim ) % _k;
   int right_node;

   // if at the right edge of the dimension, wraparound
/*
   if ( loc_in_dim == ( _k-1 ) ) {
      right_node = node - (_k-1)*k_to_dim;
   } else {
      right_node = node + k_to_dim;
   }
*/
    if ( dim == 0 ) {
      if ( node == 0 ) {
         right_node = 1;
      } else if ( node == 1 ) {
         right_node = 3;
      } else if ( node == 2 ) {
         right_node = 4;
      } else if ( node == 3 ) {
         right_node = 0;
      } else if ( node == 4 ) {
         right_node = 5;
      } else if ( node == 5 ) {
         right_node = 2;
      } else if ( node == 6 ) {
         right_node = 8;
      } else if ( node == 7 ) {
         right_node = 9;
      } else if ( node == 8 ) {
         right_node = 10;
      } else if ( node == 9 ) {
         right_node = 11;
      } else if ( node == 10 ) {
         right_node = 6;
      } else if ( node == 11 ) {
         right_node = 7;
      } else if ( node == 12 ) {
         right_node = 14;
      } else if ( node == 13 ) {
         right_node = 15;
      } else if ( node == 14 ) {
         right_node = 16;
      } else if ( node == 15 ) {
         right_node = 17;
      } else if ( node == 16 ) {
         right_node = 12;
      } else if ( node == 17 ) {
         right_node = 13;
      } else if ( node == 18 ) {
         right_node = 19;
      } else if ( node == 19 ) {
         right_node = 20;
      } else if ( node == 20 ) {
         right_node = 21;
      } else if ( node == 21 ) {
         right_node = 22;
      } else if ( node == 22 ) {
         right_node = 23;
      } else if ( node == 23 ) {
         right_node = 18;
      } else if ( node == 24 ) {
         right_node = 25;
      } else if ( node == 25 ) {
         right_node = 27;
      } else if ( node == 26 ) {
         right_node = 28;
      } else if ( node == 27 ) {
         right_node = 24;
      } else if ( node == 28 ) {
         right_node = 29;
      } else if ( node == 29 ) {
         right_node = 26;
      } else if ( node == 30 ) {
         right_node = 32;
      } else if ( node == 31 ) {
         right_node = 33;
      } else if ( node == 32 ) {
         right_node = 34;
      } else if ( node == 33 ) {
         right_node = 35;
      } else if ( node == 34 ) {
         right_node = 30;
      } else if ( node == 35 ) {
         right_node = 31;
      }
   } else if ( dim == 1 ) {
      if ( node == 0 ) {
         right_node = 6;
      } else if ( node == 1 ) {
         right_node = 19;
      } else if ( node == 2 ) {
         right_node = 20;
      } else if ( node == 3 ) {
         right_node = 21;
      } else if ( node == 4 ) {
         right_node = 22;
      } else if ( node == 5 ) {
         right_node = 11;
      } else if ( node == 6 ) {
         right_node = 12;
      } else if ( node == 7 ) {
         right_node = 13;
      } else if ( node == 8 ) {
         right_node = 14;
      } else if ( node == 9 ) {
         right_node = 15;
      } else if ( node == 10 ) {
         right_node = 16;
      } else if ( node == 11 ) {
         right_node = 17;
      } else if ( node == 12 ) {
         right_node = 18;
      } else if ( node == 13 ) {
         right_node = 31;
      } else if ( node == 14 ) {
         right_node = 32;
      } else if ( node == 15 ) {
         right_node = 33;
      } else if ( node == 16 ) {
         right_node = 34;
      } else if ( node == 17 ) {
         right_node = 23;
      } else if ( node == 18 ) {
         right_node = 24;
      } else if ( node == 19 ) {
         right_node = 25;
      } else if ( node == 20 ) {
         right_node = 26;
      } else if ( node == 21 ) {
         right_node = 27;
      } else if ( node == 22 ) {
         right_node = 28;
      } else if ( node == 23 ) {
         right_node = 29;
      } else if ( node == 24 ) {
         right_node = 30;
      } else if ( node == 25 ) {
         right_node = 1;
      } else if ( node == 26 ) {
         right_node = 2;
      } else if ( node == 27 ) {
         right_node = 3;
      } else if ( node == 28 ) {
         right_node = 4;
      } else if ( node == 29 ) {
         right_node = 35;
      } else if ( node == 30 ) {
         right_node = 0;
      } else if ( node == 31 ) {
         right_node = 7;
      } else if ( node == 32 ) {
         right_node = 8;
      } else if ( node == 33 ) {
         right_node = 9;
      } else if ( node == 34 ) {
         right_node = 10;
      } else if ( node == 35 ) {
         right_node = 5;
      }
   }
//   printf("------dim:%d, node:%d, right_node:%d\n", dim, node, right_node);

   return right_node;
}

int KNCube::GetN( ) const
{
   return _n;
}

int KNCube::GetK( ) const
{
   return _k;
}

void KNCube::InsertRandomFaults( const Configuration &config )
{
   int num_fails;
   unsigned long prev_seed;

   int node, chan;
   int i, j, t, n, c;
   bool *fail_nodes;
   bool available;

   bool edge;

   num_fails = config.GetInt( "link_failures" );

   if ( num_fails ) {
      prev_seed = RandomIntLong( );
      RandomSeed( config.GetInt( "fail_seed" ) );

      fail_nodes = new bool [_size];

      for ( i = 0; i < _size; ++i ) {
         node = i;

         // edge test
         edge = false;
         for ( n = 0; n < _n; ++n ) {
            if ( ( ( node % _k ) == 0 ) ||
                 ( ( node % _k ) == _k - 1 ) ) {
               edge = true;
            }
            node /= _k;
         }

         if ( edge ) {
            fail_nodes[i] = true;
         } else {
            fail_nodes[i] = false;
         }
      }

      for ( i = 0; i < num_fails; ++i ) {
         j = RandomInt( _size - 1 );
         available = false;

         for ( t = 0; ( t < _size ) && (!available); ++t ) {
            node = ( j + t ) % _size;

            if ( !fail_nodes[node] ) {
               // check neighbors
               c = RandomInt( 2*_n - 1 );

               for ( n = 0; ( n < 2*_n ) && (!available); ++n ) {
                  chan = ( n + c ) % 2*_n;

                  if ( chan % 1 ) {
                     available = fail_nodes[_LeftNode( node, chan/2 )];
                  } else {
                     available = fail_nodes[_RightNode( node, chan/2 )];
                  }
               }
            }

            if ( !available ) {
               cout << "skipping " << node << endl;
            }
         }

         if ( t == _size ) {
            Error( "Could not find another possible fault channel" );
         }


         OutChannelFault( node, chan );
         fail_nodes[node] = true;

         for ( n = 0; ( n < _n ) && available ; ++n ) {
            fail_nodes[_LeftNode( node, n )]  = true;
            fail_nodes[_RightNode( node, n )] = true;
         }

         cout << "failure at node " << node << ", channel " 
         << chan << endl;
      }

      delete [] fail_nodes;

      RandomSeed( prev_seed );
   }
}

double KNCube::Capacity( ) const
{
   if ( _mesh ) {
      return(double)_k / 4.0;
   } else {
      return(double)_k / 8.0;
   }
}

