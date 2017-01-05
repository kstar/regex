/***************************************************************************
                                regex.h  -  
                             -------------------
    begin                : Fri 23 Dec 2016 04:14:56 PST
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



#ifndef REGEX_H
#define REGEX_H

#include <unordered_set>
#include <stack>
#include <list>

/**
 * @class Regex
 * @short An attempt at writing something that matches regular expressions
 * @author Akarsh Simha <akarsh.simha@kdemail.net>
 */

class Regex {

private:

    /**
     * @struct NfaNode
     * @short Describes a node (i.e. state) in a non-deterministic finite automaton
     * @note This follows Ken Thomson's code
     */

    struct NfaNode {
        char ch; // 0 reserved for split, 1 reserved for accept, '.' is treated specially
        struct NfaNode *out, *out1; // Two possible outputs. out1 is valid only when this is a split.
        explicit NfaNode( char c = 0, struct NfaNode *_out = nullptr );
        explicit NfaNode( struct NfaNode *_out, struct NfaNode *_out1 );
        ~NfaNode() { --count; }
        const unsigned int id; // Identifies the NFA node; used in visualization and DFA conversion

        // Note: The difference between next_uid and count is that
        // count is decremented whenever a node is destructed,
        // next_uid is not.
        static unsigned int next_uid; // Continuously incrementing counter for UID
        static unsigned int count; // Keeps track of number of NFA nodes
    };

    /**
     * @struct NfaFragment
     * @short Describes a fragment of the NFA
     */
    struct NfaFragment {
        struct NfaNode *start;
        std::list<struct NfaNode **> outputs; // This is a list of pointers to the output pointers, so we can reassign the outputs
        NfaFragment( struct NfaNode *_start = nullptr, const std::list<struct NfaNode **> &_outputs = std::list<struct NfaNode **>() );
        NfaFragment( struct NfaNode *_start, struct NfaNode **singleOutput );
    };

    /**
     * @struct DfaNode
     * @short Describes a node (i.e. a state) in a deterministic finite automaton
     */
    struct DfaNode {
        struct DfaNode *out[256]; // Possible output transitions for each character. '.' is treated specially.
        bool accept; // This is true if this node is an accept node (i.e. a match state)
        DfaNode();
        ~DfaNode() { --count; }
        const unsigned int id;

        static unsigned int next_uid;
        static unsigned int count;
    };

public:

    /**
     * @short Constructor
     */
    Regex( const std::string &regex );

    /**
     * @short Destructor
     */
    ~Regex();

    /**
     * @short Returns a graphviz DOT format string that describes the NFA
     */
    std::string visualizeNfa( const std::string &graphName = std::string(),
                              const std::unordered_set<const NfaNode *> &highlightNodes = std::unordered_set<const NfaNode *>(),
                              const std::string &displayText = std::string() ) const;

    /**
     * @short Returns a graphviz DOT format string that describes the DFA
     */
    std::string visualizeDfa( const std::string &graphName = std::string(),
                              const std::unordered_set<const DfaNode *> &highlightNodes = std::unordered_set<const DfaNode *>(),
                              const std::string &displayText = std::string() ) const;

    /**
     * @return true if the supplied string matches the regex, false otherwise
     * @note Uses the NFA to match the string
     */
    bool nfaMatch( const std::string &str, bool visualize = false ) const;

    /**
     * @return true if the supplied string matches the regex, false otherwise
     * @note Uses the DFA to match the string
     */
    bool dfaMatch( const std::string &str ) const;

    /**
     * @return true if the supplied string matches the regex, false otherwise
     * @note Uses the generated machine code to match the string
     */
    bool match( const std::string &str ) const;

    /**
     * @return the regex
     */
    inline std::string regex() const { return m_regex; }

    /**
     * @return the postfix regex
     */
    inline std::string postfixRegex() const { return m_postfixRegex; }

private:
    std::string m_regex, m_postfixRegex;

    /**
     * @short Builds the postfix version of the regex used to build the NFA
     */
    void makePostfixRegex();

    /**
     * @short Builds the NFA from the postfix regex
     */
    void buildNfa();

    /**
     * @short Builds the DFA from the NFA by powerset construction
     */
    void buildDfa();

    /**
     * @short Generates machine code to match the regex from the DFA
     */
    void makeByteCode();

    bool isSpecial( const char ch ) const;

    void processEpsilonTransitions( std::list<const NfaNode *> &nodeList ) const;

    NfaNode *m_nfaStart;
    DfaNode *m_dfaStart;
    bool ( *m_matchFn )( const char * );
};

#endif
