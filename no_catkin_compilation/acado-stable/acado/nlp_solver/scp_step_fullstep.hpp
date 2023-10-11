/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *    \file include/acado/nlp_solver/scp_step_fullstep.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_SCP_STEP_FULLSTEP_HPP
#define ACADO_TOOLKIT_SCP_STEP_FULLSTEP_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/nlp_solver/scp_step.hpp>



BEGIN_NAMESPACE_ACADO



/**
 *	\brief Implements a fullstep to perform a step of an SCPmethod for solving NLPs.
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class SCPstepFullstep implements fullstep techniques to perform a 
 *  globalized step of an SCPmethod for solving nonlinear programming problems.
 *
 *	 \author Boris Houska, Hans Joachim Ferreau
 */
class SCPstepFullstep : public SCPstep {


    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        SCPstepFullstep( );

		SCPstepFullstep(	UserInteraction* _userInteraction
							);
					
        /** Copy constructor (deep copy). */
        SCPstepFullstep( const SCPstepFullstep& rhs );

        /** Destructor. */
        virtual ~SCPstepFullstep( );

        /** Assignment operator (deep copy). */
        SCPstepFullstep& operator=( const SCPstepFullstep& rhs );

        virtual SCPstep* clone() const;


        virtual returnValue performStep(	OCPiterate& iter,
        									BandedCP& cp,
        									SCPevaluation* eval
        									);


    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:


    //
    // DATA MEMBERS:
    //
    protected:
};


CLOSE_NAMESPACE_ACADO



//#include <acado/nlp_solver/scp_step_fullstep.ipp>


#endif  // ACADO_TOOLKIT_SCP_STEP_FULLSTEP_HPP

/*
 *  end of file
 */
