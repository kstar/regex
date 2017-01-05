/***************************************************************************
                               regex.cpp  -
                             -------------------
    begin                : Thu 29 Dec 2016 01:17:03 PST
    copyright            : (c) 2016 by Akarsh Simha
    email                : akarsh.simha@kdemail.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


/* Project Includes */
#include "regex.h"

/* STL Includes */
#include <stack>
#include <string>
#include <iostream>
#include <sstream>
#include <assert.h>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <sys/mman.h>
#include <algorithm>
#include <cstring>

unsigned int Regex::NfaNode::count = 0;
unsigned int Regex::NfaNode::next_uid = 0;
unsigned int Regex::DfaNode::count = 0;
unsigned int Regex::DfaNode::next_uid = 0;

Regex::Regex(const std::string& regex) : m_regex( regex ) {
    assert( !regex.empty() );
    m_nfaStart = nullptr;
    m_dfaStart = nullptr;
    m_matchFn = nullptr;
    makePostfixRegex();
    buildNfa();
    buildDfa();
    makeByteCode();
}

bool Regex::isSpecial( const char ch ) const {
        if ( ch == '*' || ch == '.' || ch == '!' || ch == '?' || ch == '+'
             || ch == '(' || ch == ')' || ch == '|' )
            return true;
        return false;
};

void Regex::makePostfixRegex() {
    // We use '!' as the concatenation operator, it better not be in the regex string
    assert( m_regex.find( '!' ) == std::string::npos );
    std::stack<char> opStack;
    bool prevExp = false;
    for ( int j = 0; j < m_regex.size(); ++j ) {
        const char ch = m_regex[j];
        if ( !isSpecial( ch ) || ch == '.' ) {
            m_postfixRegex.push_back( ch );
            if ( prevExp )
                opStack.push( '!' ); // push concatenation operator
            prevExp = true;
            continue;
        }
        switch( ch ) {
        case '(':
            // When we encounter an open paren, we should push it on
            // the stack and continue
            if ( prevExp )
                opStack.push( '!' );
            prevExp = false;
            opStack.push( '(' );
            break;
        case '+':
        case '?':
        case '*':
            // These repetition operators are anyway postfix, and have
            // the strongest binding, so we just put them in the
            // result as it is.
            assert( prevExp );
            m_postfixRegex.push_back( ch );
            break;
        case '|':
        case ')':
            // Alternation has the weakest binding, so we flush the
            // stack of all higher-precedence operations and then push
            // the alternation operator
            assert( prevExp );
            while ( !opStack.empty() && opStack.top() != '(' ) {
                m_postfixRegex.push_back( opStack.top() );
                opStack.pop();
            }
            if ( ch == ')' ) {
                assert( opStack.top() == '(' );
                opStack.pop();
            }
            else if ( ch == '|' ) {
                opStack.push( '|' );
            }
            prevExp = ( ch == ')' );
            break;
        default:
            assert( false );
        }
    }
    while ( !opStack.empty() ) {
        m_postfixRegex.push_back( opStack.top() );
        opStack.pop();
    }
}


void Regex::buildNfa() {
    // Take the post-fix regex and build an automaton that accepts the
    // language described by the regex

    std::stack<NfaFragment *> fragStack;

    auto take = [ &fragStack ]() {
        NfaFragment *ret = fragStack.top();
        fragStack.pop();
        return ret;
    };

    auto patch = []( NfaFragment *frag, NfaNode *state ) -> void {
        for ( NfaNode **output : frag->outputs )
            *output = state;
    };

    for ( const char ch : m_postfixRegex ) {
        assert( ch != '(' && ch != ')' );
        if ( !isSpecial( ch ) || ch == '.' ) {
            NfaNode *myNode = new NfaNode( ch );
            NfaFragment *thisFrag = new NfaFragment( myNode, &( myNode->out ) );
            fragStack.push( thisFrag );
            continue;
        }
        assert( !fragStack.empty() ); // cannot use operator without literal
        NfaFragment *lastFrag = take();
        switch( ch ) {
        case '!': { // concatenation
            NfaFragment *secondLastFrag = take();
            patch( secondLastFrag, lastFrag->start );
            NfaFragment *catFrag = new NfaFragment( secondLastFrag->start, lastFrag->outputs );
            fragStack.push( catFrag );
            delete lastFrag;
            delete secondLastFrag;
            break;
        }
        case '|': {
            NfaFragment *secondLastFrag = take();
            NfaNode *splitNode = new NfaNode( secondLastFrag->start, lastFrag->start );
            NfaFragment *altFrag = new NfaFragment( splitNode, secondLastFrag->outputs );
            altFrag->outputs.splice( altFrag->outputs.end(), lastFrag->outputs );

            fragStack.push( altFrag );
            delete lastFrag;
            delete secondLastFrag;
            break;
        }
        case '?': {
            NfaNode *splitNode = new NfaNode( lastFrag->start, nullptr );
            NfaFragment *qFrag = new NfaFragment( splitNode, lastFrag->outputs );
            qFrag->outputs.push_back( &( splitNode->out1 ) );

            fragStack.push( qFrag );
            delete lastFrag;
            break;
        }
        case '*': {
            NfaNode *splitNode = new NfaNode( lastFrag->start, nullptr );
            NfaFragment *aFrag = new NfaFragment( splitNode,  &( splitNode->out1 ) );
            patch( lastFrag, splitNode );

            fragStack.push( aFrag );
            delete lastFrag;
            break;
        }
        case '+': {
            // This case may be subsumed into ! and * but...
            NfaNode *splitNode = new NfaNode( lastFrag->start, nullptr );
            patch( lastFrag, splitNode );
            NfaFragment *pFrag = new NfaFragment( lastFrag->start, &( splitNode->out1 ) );

            fragStack.push( pFrag );
            delete lastFrag;
            break;
        }
        default:
            assert( false );
        }

    }

    assert( fragStack.size() == 1 );
    NfaFragment *fullNfa = take();
    NfaNode *acceptNode = new NfaNode( 1 );
    patch( fullNfa, acceptNode );
    m_nfaStart = fullNfa->start;
    delete fullNfa;
}


bool Regex::dfaMatch( const std::string& str ) const {
    if ( str.empty() )
        return false;

    assert( m_dfaStart );
    DfaNode *current = m_dfaStart;

    for ( const char ch : str ) {
        if ( current->out[ ch ] )
            current = current->out[ ch ];
        else if ( current->out[ '.' ] )
            current = current->out[ '.' ];
        else
            return false;
    }
    if ( current->accept )
        return true;
}

bool Regex::match( const std::string &str ) const {
    assert( m_matchFn );
    return m_matchFn( str.c_str() );
}

Regex::~Regex() {
    auto delNfa = [this]() {
        std::unordered_set<NfaNode*> delSet;
        std::queue<NfaNode*> bfsQueue;
        bfsQueue.push( m_nfaStart );
        while ( !bfsQueue.empty() ) {
            NfaNode *n = bfsQueue.front();
            bfsQueue.pop();
            if ( delSet.count( n ) )
                continue;
            if ( n->out )
                bfsQueue.push( n->out );
            if ( n->out1 && n->out1 != n )
                bfsQueue.push( n->out1 );
            delSet.insert( n );
        }
        for ( NfaNode *node : delSet )
            delete node;
    };

    auto delDfa = [this]() {
        std::unordered_set<DfaNode*> delSet;
        std::queue<DfaNode*> bfsQueue;
        bfsQueue.push( m_dfaStart );
        while ( !bfsQueue.empty() ) {
            DfaNode *n = bfsQueue.front();
            bfsQueue.pop();
            if ( delSet.count( n ) )
                continue;
            delSet.insert( n );
            for ( int i = 0; i < 256; ++i )
                if ( n->out[i] )
                    bfsQueue.push( n->out[i] );
        }
        for ( DfaNode *node : delSet )
            delete node;
    };

    auto delByteCode = [this]() {
        munmap( ( void * )( m_matchFn ), 4096 );
    };

    delNfa(); delDfa(); delByteCode();
}

Regex::NfaNode::NfaNode( char c, struct NfaNode *_out ) : ch( c ), out( _out ), out1( nullptr ), id( next_uid++ ) {
    ++count;
}

Regex::NfaNode::NfaNode(struct NfaNode* _out, struct NfaNode* _out1) : ch( 0 ), out( _out ), out1( _out1 ), id( next_uid++ ) {
    ++count;
}

Regex::NfaFragment::NfaFragment( struct NfaNode* _start, const std::list<struct NfaNode**> &_outputs ) : start( _start ), outputs( _outputs ) {
}


Regex::NfaFragment::NfaFragment(struct NfaNode* _start, struct NfaNode** singleOutput) : start( _start ) {
    assert( singleOutput );
    outputs.push_back( singleOutput );
}


std::string Regex::visualizeNfa( const std::string &graphName,  const std::unordered_set<const NfaNode *> &highlightNodes, const std::string &displayText ) const {
    std::ostringstream dot;

    // Code to visualize NFA
    dot << "digraph ";
    if ( graphName.empty() )
        dot << "g";
    else
        dot << graphName;
    dot << " {" << std::endl;

    if ( !displayText.empty() ) {
        dot << "N [label = <" << displayText << ">, shape=plaintext, pos=\"1,1!\"]" << std::endl;
    }
    std::queue<NfaNode *> q;
    std::unordered_set<NfaNode *> visited;
    auto name = []( const NfaNode *node ) { return node->id; };
    q.push( m_nfaStart );
    dot << name( m_nfaStart ) << " [style=\"solid, filled\", fillcolor=green]" << std::endl;
    while ( !q.empty() ) {
        NfaNode *me = q.front();
        q.pop();
        if ( visited.count( me ) )
            continue;
        visited.insert( me );
        if ( me->ch == 1 ) {
            dot << name( me ) << " [style=\"solid, filled\", fillcolor=red]" << std::endl;
            continue;
        }
        if ( highlightNodes.count( me ) )
            dot << name( me ) << " [style=\"solid, filled\", fillcolor=orange]" << std::endl;
        assert( me->out );
        if ( me->out1 ) {
            // split
            assert( me->ch == 0 );
            dot << name( me ) << " -> " << name( me->out1 ) << " [label = \"ɛ\"]" << std::endl;
            q.push( me->out1 );
            dot << name( me ) << " -> " << name( me->out ) << " [label = \"ɛ\"]" << std::endl;
        }
        else {
            dot << name( me ) << " -> " << name( me->out ) << " [label = \"";
            if ( me->ch > 32 )
                dot << me->ch;
            else
                dot << "0x" << std::hex << int( me->ch ) << "" << std::dec;
            dot << "\"]" << std::endl;
        }
        q.push( me->out );
    }
    dot << "}" << std::endl;


    return dot.str();
}

std::string Regex::visualizeDfa( const std::string &graphName,  const std::unordered_set<const DfaNode *> &highlightNodes, const std::string &displayText ) const {
    std::ostringstream dot;

    // Code to visualize DFA
    dot << "digraph ";
    if ( graphName.empty() )
        dot << "g";
    else
        dot << graphName;
    dot << " {" << std::endl;

    if ( !displayText.empty() ) {
        dot << "N [label = <" << displayText << ">, shape=plaintext, pos=\"1,1!\"]" << std::endl;
    }
    std::queue<DfaNode *> q;
    std::unordered_set<DfaNode *> visited;
    auto name = []( const DfaNode *node ) { return node->id; };
    q.push( m_dfaStart );
    dot << name( m_dfaStart ) << " [style=\"solid, filled\", fillcolor=green]" << std::endl;
    while ( !q.empty() ) {
        DfaNode *me = q.front();
        q.pop();
        if ( visited.count( me ) )
            continue;
        visited.insert( me );

        if ( me->accept )
            dot << name( me ) << " [style=\"solid, filled\", fillcolor=red]" << std::endl;

        if ( highlightNodes.count( me ) )
            dot << name( me ) << " [style=\"solid, filled\", fillcolor=orange]" << std::endl;

        for ( int i = 0; i < 256; ++i ) {
            if ( me->out[i] ) {
                dot << name( me ) << " -> " << name( me->out[i] ) << " [label = \"";
                if ( i > 32 )
                    dot << char( i );
                else
                    dot << "0x" << std::hex << i << "" << std::dec;
                dot << "\"]" << std::endl;
                q.push( me->out[i] );
            }
        }

    }
    dot << "}" << std::endl;


    return dot.str();
}


bool Regex::nfaMatch( const std::string& str, bool visualize ) const {
    std::list<const NfaNode *> frontier, next;

    auto visualizeFrontier = [ &frontier, &str, this ]( const int i ) {
        std::unordered_set<const NfaNode *> highlightNodes;
        for ( const NfaNode *node : frontier )
            highlightNodes.insert( node );
        std::cout << visualizeNfa( "step" + std::to_string( i ), highlightNodes, str.substr( 0, i ) + "<B>" + str.substr( i, 1 ) + "</B>" + str.substr( i + 1 ) );
        std::cout << std::endl;
    };

    /*
     * Simulate the NFA by keeping track of all possible current states
     * This looks a bit like a BFS, but is not, in that we follow loopy arrows
     * The 'frontier' consists of the current set of nodes we could be in
     * The 'next' list consists of the set of nodes we can transition into
     */
    frontier.push_back( m_nfaStart );
    for ( int i = 0; i < str.length(); ++i ) {
        const char ch = str[i];

        if ( frontier.empty() )
            return false;

        processEpsilonTransitions( frontier );

        visualizeFrontier( i );

        auto it = frontier.begin();
        while ( it != frontier.end() ) {
            const NfaNode *frontierState = ( *it );
            if ( frontierState->ch == '.' || frontierState->ch == ch ) {
                // Valid transition
                next.push_back( frontierState->out );
                assert( !frontierState->out1 ); // should not be a split
            }
            it = frontier.erase( it );
        }
        frontier.swap( next );
    }

    // Find any accept states
    processEpsilonTransitions( frontier );

    for ( const NfaNode *state : frontier ) {
        if ( state->ch == 1 )
            return true;
    }

    return false;
}


void Regex::processEpsilonTransitions( std::list<const NfaNode *> &nodeList ) const {

    // This method processes all epsilon transitions in frontier iteratively

    auto it = nodeList.begin();
    while ( it != nodeList.end() ) {
        const NfaNode *me = ( *it );
        if ( me->ch != 0 ) {
            ++it;
            continue;
        }

        // We have encountered a split
        assert( me->out && me->out1 ); // check
        nodeList.push_back( me->out ); // add the new states...
        nodeList.push_back( me->out1 ); // ... at the back, so we can further transition when we visit them
        it = nodeList.erase( it ); // and move the iterator
    }

}

void Regex::buildDfa() {

    /*
     * This method builds the DFA corresponding to the NFA
     * Each DFA state corresponds to a unique set of NFA states
     * The DFA states are identified by a bit-map representation of sets of NFA states
     * The NFA is explored using breadth-first search, and DFA states are constructed for each unique combination of NFA states
     * The DFA states are then connected.
     * The connections are made such that a '.' transition is processed only if there exists no proper transition for the given character, endowing the DFA with determinism.
     */

    assert( m_nfaStart );
    m_dfaStart = nullptr;

    typedef std::vector<bool> DfaNodeId; // Each state in the DFA is identified by the set of NFA states it corresponds to. The set is represented as a bit-map.
    std::unordered_map<DfaNodeId, DfaNode *> dfaMap; // This map maps a DFA identified by a set of NFA states to the unique struct DfaNode it corresponds to.

    const int N = NfaNode::next_uid; // Hoping that NfaNode::next_uid is not largely different from NfaNode::count, we take the liberty of making a set of size next_uid
    auto getDfaNodeId = [N]( const std::list<const NfaNode *> &nfaNodeList ) -> DfaNodeId {
        // Convert a list of NFA nodes to the bit-map representation of the same set
        DfaNodeId myId( N, false );
        for ( const NfaNode *nfaNode : nfaNodeList )
            myId[ nfaNode->id ] = true;
        return myId;
    };

    auto getDfaNode = [&getDfaNodeId, &dfaMap]( const std::list<const NfaNode *> &nfaNodeList ) -> DfaNode * {
        // Obtain the DFA node corresponding to the given bit-map identifier, creating a new node if it does not exist
        const auto &myId = getDfaNodeId( nfaNodeList );
        if ( dfaMap.count( myId ) )
            return dfaMap[ myId ];
        else
            return ( dfaMap[ myId ] = new DfaNode );
    };

    // Traverse the NFA and construct the DFA
    std::unordered_set<DfaNode *> visited;
    std::queue<std::list<const NfaNode *>> bfsQueue;
    std::list<const NfaNode *> nfaNodeList;
    nfaNodeList.push_back( m_nfaStart );
    processEpsilonTransitions( nfaNodeList );
    bfsQueue.push( nfaNodeList );
    m_dfaStart = getDfaNode( nfaNodeList );
    while ( !bfsQueue.empty() ) {
        std::list<const NfaNode *> &listState = bfsQueue.front();

        DfaNode *state = getDfaNode( listState );
        if ( visited.count( state ) ) {
            bfsQueue.pop();
            continue;
        }
        visited.insert( state );

        // Explore all possible transitions
        std::unordered_map<char, std::unordered_set<const NfaNode *>> transitionMap;
        for ( const NfaNode *node : listState ) {
            assert( node->ch != 0 ); // epsilon transitions must've been processed
            if ( node->ch == 1 ) {
                state->accept = true;
            }
            else {
                assert( node->out );
                transitionMap[ node->ch ].insert( node->out );
            }

        }

        // Duplicate . transitions in each non-dot transition
        // So the DFA execution will check as follows:
        //    if ( input char exists as a transition )
        //        make the transition
        //    else if( there is a dot transition )
        //        make the dot transition
        //    else
        //        fail match
        if ( transitionMap.count( '.' ) ) {
            for ( auto &item : transitionMap )
                if ( item.first != '.' )
                    for ( const NfaNode *dotDest : transitionMap['.'] )
                        item.second.insert( dotDest );
        }

        // Make a DFA node for each possible transition,
        // and also add each such set of NFA states as a list into the BFS queue
        for ( auto &item : transitionMap ) {
            const char ch = item.first;
            assert( ch != 0 );
            const std::unordered_set<const NfaNode *> &destSet = item.second;
            bfsQueue.emplace();
            std::list<const NfaNode *> &destList = bfsQueue.back();
            for ( const NfaNode *node : destSet )
                destList.push_back( node );
            processEpsilonTransitions( destList );
            DfaNode *destState = getDfaNode( destList );
            state->out[ ch ] = destState;
        }

        bfsQueue.pop();
    }

}

void Regex::makeByteCode() {
    /*
     * This method turns the DFA into machine code that simulates the
     * DFA. The machine code generated is for x86_64 Intel processors
     * using System V ABI calling convention.
     *
     * The machine code is designed to work as a function call -- the
     * pointer to the null-terminated C string is accepted as an
     * argument into register rdi. The result is a bool returned in
     * eax. This is as per System V calling conventions for x86_64
     * machines.
     *
     * For each generic state, we generate the following code:
     * fetch the next character
     *     add rdi, 1
     *     mov al, [rdi]
     * if the state is an accept state, insert code to process acceptance
     *     cmp al, 0
     *     jne notaccepted
     *     mov eax, 1
     *     ret
     *   notaccepted:
     * then, for each possible transition:
     *   cmp al, <transition character>
     *   je <destination of transition>
     * then, for any dot transitions:
     *   cmp al, 0
     *   jne <destination of dot transition>
     * if we cannot transition out of the state, return false:
     *   mov eax, 0
     *   ret
     *
     * For the first entry into the start state alone, we have the
     * following additional code:
     *     mov rax, 0
     *     jmp 4
     * the 4 byte jump skips the add rdi, 1 instruction that advances
     * the string pointer. Subsequent re-entries into the start state
     * do not skip the add rdi, 1.
     *
     * It is non-trivial to pre-compute the offsets correctly, so we
     * just fill in 32-bit 0x0 for the offset, and later replace it
     * with the correct offsets for each state. The way this works is
     * that we maintain a vector of offsets from the beginning of the
     * start state code for each state (indexed by DfaNode::id). We do
     * a BFS through the DFA, and for each state we encounter, we
     * write down its offset relative to the start state (so the start
     * state entry point, i.e. the add rdi, 1 instruction, has an
     * offset of 0) into the vector. For each jump to a state, we make
     * a note of the location of the jump instruction, and the
     * corresponding id of the destination state of the jump in the
     * map jeMap. After the full BFS is complete, we go back and write
     * in the correct offsets from the EIP at the jump instruction to
     * the beginning of the code for the destination state.
     */
    typedef unsigned int StateId;
    std::vector<int> offsets( Regex::DfaNode::count, -1 ); // Maps states to their offsets in the code
    std::unordered_map<int, StateId> jeMap; // Maps je instruction offsets to state IDs
    char * const m = ( char * ) mmap( 0, 4096, 3, 0x22, -1, 0 ); // FIXME: Change 4096 ->
    std::copy_n( "\x48\xb8\x00\x00\x00\x00\x00\x00\x00\x00", 10, m ); // Clear rax
    std::copy_n( "\xeb\x04", 2, m + 10 ); // jmp 4 : required to skip the add rdi, 1 the first time
    char * const start = m + 12; // Start state beginning in the function's code (corresponds to 0 'offset')
    int offset = 0;

    // Lay out DFA nodes in bytecode, without knowing the jump offsets
    // -- use a breadth first search to explore the DFA.
    std::queue<const DfaNode *> bfsQueue;
    std::unordered_set<const DfaNode *> visited;
    bfsQueue.push( m_dfaStart );
    auto write = [&start, &offset]( const char *bytecode, int N ) {
        std::copy_n( bytecode, N, start + offset );
        offset += N;
    };
    while ( !bfsQueue.empty() ) {
        const DfaNode * const me = bfsQueue.front();
        bfsQueue.pop();
        if ( visited.count( me ) )
            continue;
        visited.insert( me );

        /* Note down the offset (from start state entry point) for this state */
        offsets[ me->id ] = offset;

        /* Write code to fetch the next character */
        write( "\x48\x83\xc7\x01", 4 ); // add rdi, 1 : advances to next char
        write( "\x8a\x07", 2 ); // mov al, [rdi] : gets a byte from the string
        bool dot = false;

        if ( me->accept ) {
            /* Write code to check for string termination in the accept state, and return true if this happens */
            write( "\x3c\x00", 2 ); // cmp al, 0 : check if we are looking at a null terminator
            write( "\x75\x06", 2 ); // jne 6 : if not, jump 6 bytes to circumvent the following return true
            write( "\xb8\x01\x00\x00\x00", 5 ); // mov eax, 1 : move 'true' into eax (return value)
            write( "\xc3", 1 ); // ret : return
        }
        for ( int ch = 0; ch < 256; ++ch ) {
            if ( me->out[ch] ) {
                if ( ch == '.' ) {
                    /* We process dot transitions at the end, since they are the 'last resort' (see buildDfa() for more) */
                    dot = true;
                    continue;
                }
                else {
                    /* Write code to check transition, and jump as appropriate.
                     * Make a note of the offset of je instruction so we can later write in the correct jump address */
                    write( "\x3c", 1 ); start[ offset++ ] = ch; // cmp al, ch
                    jeMap[offset] = me->out[ch]->id; // Make a note of the offset of the jump so we can enter the right rel32 address later
                    write( "\x0f\x84\x00\x00\x00\x00", 6 ); // je <offset to be replaced>
                    bfsQueue.push( me->out[ch] );
                }
            }
        }
        if ( dot ) { // dot transition should be the last examined, hence outside loop
            /* Write code to process dot transition if we have not hit a null character */
            write( "\x3c\x00", 2 ); // cmp al, 0 : if we have not hit the null character, use the dot transition
            jeMap[offset] = me->out['.']->id;
            write( "\x0f\x85\x00\x00\x00\x00", 6 ); // jne <offset to be replaced>
            bfsQueue.push( me->out['.'] );
        }

        /* Write failure code -- if we could not transition out of this state, we return false */
        write( "\xb8\x00\x00\x00\x00", 5 ); // mov eax, 0
        write( "\xc3", 1 ); // ret
    }

    // Fix the connections : replace the rel32 offsets of the jump instructions with the correct offsets
    for ( const auto &jePair : jeMap ) {
        const int destOffset = offsets[ jePair.second ];
        const int jeOffset = jePair.first;
        assert( destOffset >= 0 );
        int joff = destOffset - ( jeOffset + 6 ); // + 6 because the jump instruction is 6 bytes long, and RIP points after that.
        std::memcpy( start + jeOffset + 2, &joff, 4 ); // + 2 because of the first two bytes of the jump instruction (2-byte opcode)
    }

    mprotect( m, 4096, 4 ); // Make memory executable
    m_matchFn = ( bool( * )( const char * ) )m; // Cast into function pointer
    std::cerr << "Generated " << offset + 5 << " bytes of machine code." << std::endl;
}

Regex::DfaNode::DfaNode() : accept( false ), id( next_uid++ ) {
    ++count;
    for ( int i = 0; i < 256; ++i )
        out[i] = nullptr;
}
