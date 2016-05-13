@routing @guidance @turn-lanes
Feature: Turn Lane Guidance

    Background:
        Given the profile "car"
        Given a grid size of 20 meters

    #requires https://github.com/cucumber/cucumber-js/issues/417
    #Due to this, we use & as a pipe character. Switch them out for \| when 417 is fixed
    @bug @WORKAROUND-FIXME
    Scenario: Basic Turn Lane 3-way Turn with empty lanes
        Given the node map
            | a |   | b |   | c |
            |   |   | d |   |   |

        And the ways
            | nodes  | turn:lanes     | turn:lanes:forward | turn:lanes:backward | name     |
            | ab     |                | through\|right     |                     | in       |
            | bc     |                |                    | left\|through&&     | straight |
            | bd     |                |                    | left\|right         | right    |

       When I route I should get
            | waypoints | route                | turns                           | lanes   |
            | a,c       | in,straight,straight | depart,new name straight,arrive | ,1,     |
            | a,d       | in,right,right       | depart,turn right,arrive        | ,0,     |
            | c,a       | straight,in,in       | depart,new name straight,arrive | ,0 1 2, |
            | c,d       | straight,right,right | depart,turn left,arrive         | ,3,     |

    Scenario: Basic Turn Lane 4-Way Turn
        Given the node map
            |   |   | e |   |   |
            | a |   | b |   | c |
            |   |   | d |   |   |

        And the ways
            | nodes  | turn:lanes     | turn:lanes:forward | turn:lanes:backward | name     |
            | ab     |                | \|right            |                     | in       |
            | bc     |                |                    |                     | straight |
            | bd     |                |                    | left\|              | right    |
            | be     |                |                    |                     | left     |

       When I route I should get
            | waypoints | route                   | turns                           | lanes |
            | a,c       | in,straight,straight    | depart,new name straight,arrive | ,1,   |
            | a,d       | in,right,right          | depart,turn right,arrive        | ,0,   |
            | a,e       | in,left,left            | depart,turn left,arrive         | ,1,   |
            | d,a       | right,in,in             | depart,turn left,arrive         | ,1,   |
            | d,e       | right,left,left         | depart,new name straight,arrive | ,0,   |
            | d,c       | right,straight,straight | depart,turn right,arrive        | ,0,   |

    Scenario: Basic Turn Lane 4-Way Turn using none
        Given the node map
            |   |   | e |   |   |
            | a |   | b |   | c |
            |   |   | d |   |   |

        And the ways
            | nodes  | turn:lanes     | turn:lanes:forward | turn:lanes:backward | name     |
            | ab     |                | none\|right        |                     | in       |
            | bc     |                |                    |                     | straight |
            | bd     |                |                    | left\|none          | right    |
            | be     |                |                    |                     | left     |

       When I route I should get
            | waypoints | route                | turns                           | lanes |
            | a,c       | in,straight,straight | depart,new name straight,arrive | ,1,   |
            | a,d       | in,right,right       | depart,turn right,arrive        | ,0,   |
            | a,e       | in,left,left         | depart,turn left,arrive         | ,1,   |

    Scenario: Basic Turn Lane 4-Way WWith U-Turn Lane
        Given the node map
            |   |   | e |   |   |
            | a |   | b |   | c |
            |   |   | d |   |   |

        And the ways
            | nodes  | turn:lanes     | turn:lanes:forward  | name     |
            | ab     |                | reverse;left\|right | in       |
            | bc     |                |                     | straight |
            | bd     |                |                     | right    |
            | be     |                |                     | left     |

       When I route I should get
            | waypoints | route                | turns                           | lanes |
            | a,c       | in,straight,straight | depart,new name straight,arrive | ,1,   |
            | a,d       | in,right,right       | depart,turn right,arrive        | ,0,   |
            | a,e       | in,left,left         | depart,turn left,arrive         | ,1,   |


    #this next test requires decision on how to announce lanes for going straight if there is no turn
    @TODO @WORKAROUND-FIXME
    Scenario: Turn with Bus-Lane
        Given the node map
            | a |   | b |   | c |
            |   |   |   |   |   |
            |   |   | d |   |   |

        And the ways
            | nodes | name | turn:lanes:forward | lanes:psv:forward |
            | ab    | road | through\|right&    | 1                 |
            | bc    | road |                    |                   |
            | bd    | turn |                    |                   |

        When I route I should get
            | waypoints | route          | turns                           | lanes |
            | a,d       | road,turn,turn | depart,turn right,arrive        | ,0,   |
            | a,c       | road,road,road | depart,use lane straight,arrive | ,1,   |

    #turn lanes are often drawn at the incoming road, even though the actual turn requires crossing the intersection first
    @TODO @WORKAROUND-FIXME
    Scenario: Turn Lanes at Segregated Road
        Given the node map
            |   |   | i | l |   |   |
            |   |   |   |   |   |   |
            | h |   | g | f |   | e |
            | a |   | b | c |   | d |
            |   |   |   |   |   |   |
            |   |   | j | k |   |   |

        And the ways
            | nodes | name  | turn:lanes:forward      | oneway |
            | ab    | road  | left\|through&right     | yes    |
            | bc    | road  | left\|through           | yes    |
            | cd    | road  |                         | yes    |
            | ef    | road  | \|through&through;right | yes    |
            | fg    | road  | left;through\|through&  | yes    |
            | gh    | road  |                         | yes    |
            | ig    | cross |                         | yes    |
            | gb    | cross | left\|through           | yes    |
            | bj    | cross |                         | yes    |
            | kc    | cross | left\|through;right     | yes    |
            | cf    | cross | left\|through           | yes    |
            | fl    | cross |                         | yes    |

        When I route I should get
            | waypoints | route             | turns                           | lanes | #                          |
            | a,j       | road,cross,cross  | depart,turn right,arrive        | ,0,   |                            |
            | a,d       | road,road,road    | depart,use lane straight,arrive | ,1,   | #post-processing reduction |
            | a,l       | road,cross,cross  | depart,turn left,arrive         | ,2,   |                            |
            | a,h       | road,road,road    | depart,continue uturn,arrive    | ,2,   |                            |
            | k,d       | cross,road,road   | depart,turn right,arrive        | ,0,   |                            |
            | k,l       | cross,cross,cross | depart,use lane straight,arrive | ,0,   |                            |
            | k,h       | cross,road,road   | depart,turn left,arrive         | ,1,   |                            |
            | k,j       | cross,cross,cross | depart,continue uturn,arrive    | ,1,   |                            |
            | e,l       | road,cross,cross  | depart,turn right,arrive        | ,0,   |                            |
            | e,h       | road,road         | depart,arrive                   | ,     |                            |
            | e,j       | road,cross,cross  | depart,turn left,arrive         | ,2,   |                            |
            | e,d       | road,road,road    | depart,continue uturn,arrive    | ,2,   |                            |
            | i,h       | cross,road,road   | depart,turn right,arrive        | ,,    |                            |
            | i,j       | cross,cross,cross | depart,use lane straight,arrive | ,0,   |                            |
            | i,d       | cross,road,road   | depart,turn left,arrive         | ,1,   |                            |
            | i,l       | cross,cross,cross | depart,continue uturn,arrive    | ,1,   |                            |

    Scenario: Turn Lanes at Segregated Road
        Given the node map
            |   |   | g | f |   |   |
            | a |   | b | c |   | d |
            |   |   |   |   |   |   |
            |   |   | j | k |   |   |

        And the ways
            | nodes | name  | turn:lanes:forward      | oneway |
            | ab    | road  | left\|through&right     | yes    |
            | bc    | road  |                         | yes    |
            | cd    | road  |                         | yes    |
            | gb    | cross |                         | yes    |
            | bj    | cross |                         | yes    |
            | kc    | cross |                         | yes    |
            | cf    | cross |                         | yes    |

        When I route I should get
            | waypoints | route             | turns                           | lanes |
            | a,j       | road,cross,cross  | depart,turn right,arrive        | ,0,   |

    #this can happen due to traffic lights / lanes not drawn up to the intersection itself
    Scenario: Turn Lanes Given earlier than actual turn
        Given the node map
            | a |   | b | c |   | d |
            |   |   |   |   |   |   |
            |   |   |   | e |   |   |

        And the ways
            | nodes | name | turn:lanes:forward |
            | ab    | road | \|right            |
            | bc    | road |                    |
            | cd    | road |                    |
            | ce    | turn |                    |

        When I route I should get
            | waypoints | route          | turns                           | lanes |
            | a,e       | road,turn,turn | depart,turn right,arrive        | ,0,   |
            | a,d       | road,road,road | depart,use lane straight,arrive | ,1,   |

    Scenario: Turn Lanes Given earlier than actual turn
        Given the node map
            | a |   | b | c | d |   | e |   | f | g | h |   | i |
            |   |   | j |   |   |   |   |   |   |   | k |   |   |

        And the ways
            | nodes | name        | turn:lanes:forward | turn:lanes:backward |
            | abc   | road        |                    |                     |
            | cd    | road        |                    | left\|              |
            | def   | road        |                    |                     |
            | fg    | road        | \|right            |                     |
            | ghi   | road        |                    |                     |
            | bj    | first-turn  |                    |                     |
            | hk    | second-turn |                    |                     |

        When I route I should get
            | waypoints | route                        | turns                           | lanes |
            | a,k       | road,second-turn,second-turn | depart,turn right,arrive        | ,0,   |
            | a,i       | road,road,road               | depart,use lane straight,arrive | ,1,   |
            | i,j       | road,first-turn,first-turn   | depart,turn left,arrive         | ,1,   |
            | i,a       | road,road,road               | depart,use lane straight,arrive | ,0,   |

    Scenario: Passing a one-way street
        Given the node map
            | e |   |   | f |   |
            | a |   | b | c | d |

        And the ways
            | nodes | name | turn:lanes:forward | oneway |
            | ab    | road | left\|through      | no     |
            | bcd   | road |                    | no     |
            | eb    | owi  |                    | yes    |
            | cf    | turn |                    |        |

        When I route I should get
            | waypoints | route          | turns                   | lanes |
            | a,f       | road,turn,turn | depart,turn left,arrive | ,1,   |

    Scenario: Passing a one-way street, partly pulled back lanes
        Given the node map
            | e |   |   | f |   |
            | a |   | b | c | d |
            |   |   | g |   |   |

        And the ways
            | nodes | name  | turn:lanes:forward  | oneway |
            | ab    | road  | left\|through;right | no     |
            | bcd   | road  |                     | no     |
            | eb    | owi   |                     | yes    |
            | cf    | turn  |                     | no     |
            | bg    | right |                     | no     |

        When I route I should get
            | waypoints | route            | turns                    | lanes |
            | a,f       | road,turn,turn   | depart,turn left,arrive  | ,1,   |
            | a,g       | road,right,right | depart,turn right,arrive | ,0,   |

    Scenario: Passing a one-way street, partly pulled back lanes, no through
        Given the node map
            | e |   |   | f |
            | a |   | b | c |
            |   |   | g |   |

        And the ways
            | nodes | name  | turn:lanes:forward  | oneway |
            | ab    | road  | left\|right         | no     |
            | bc    | road  |                     | no     |
            | eb    | owi   |                     | yes    |
            | cf    | turn  |                     | no     |
            | bg    | right |                     | no     |

        When I route I should get
            | waypoints | route            | turns                    | lanes |
            | a,f       | road,turn,turn   | depart,turn left,arrive  | ,1,   |
            | a,g       | road,right,right | depart,turn right,arrive | ,0,   |

     Scenario: Narrowing Turn Lanes
        Given the node map
            |   |   |   |   | g |   |
            |   |   |   |   |   |   |
            | a |   | b | c | d | e |
            |   |   |   | f |   |   |

        And the ways
            | nodes | name    | turn:lanes:forward  |
            | ab    | road    | left\|through&right |
            | bc    | road    |                     |
            | cd    | road    | left\|through       |
            | de    | through |                     |
            | dg    | left    |                     |
            | cf    | right   |                     |

        When I route I should get
            | waypoints | route                | turns                           | lanes |
            | a,g       | road,left,left       | depart,turn left,arrive         | ,2,   |
            | a,e       | road,through,through | depart,new name straight,arrive | ,1,   |
            | a,f       | road,right,right     | depart,turn right,arrive        | ,0,   |

    Scenario: Anticipate Lane Change
        Given the node map
            | a |   | b |   | x |
            |   |   |   |   |   |
            |   |   | c |   | d |
            |   |   |   |   |   |
            |   |   | y |   |   |

        And the ways
            | nodes | turn:lanes:forward   | turn:lanes:backward |
            | ab    | through\|right&right |                     |
            | bx    |                      | left\|left&through  |
            | bc    | left\|through        | left\|right         |
            | cd    |                      | left\|right         |
            | cy    |                      |                     |

       When I route I should get
            | waypoints | route       | turns                                            | lanes |
            | d,a       | cd,bc,ab,ab | depart,end of road right,end of road left,arrive | ,0,1, |
