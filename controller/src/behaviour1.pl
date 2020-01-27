:- dynamic(list/1).
:- dynamic(degree/1).
:- dynamic(perception/1).

% Per come è fatto adesso asserziona le percezioni passate e con takeDecision sceglie dove andare.
% perception(['False', 'False', 'False', 'False'])
% False= Non ho ostacolo;
% True = Ho un ostacolo;
hasValue('NorthSensor', S1) :- perception([S1, _, _, _]).
hasValue('WestSensor', S2) :- perception([_, S2, _, _]).
hasValue('EastSensor', S3) :- perception([_, _, S3, _]).
hasValue('SouthSensor', S4) :- perception([_, _, _, S4]).


degree(0). % Per contenere orientamento.

list([]). % Per le direzioni precedentemente prese.
% list(['W', 'W', 'S', 'N']).

count([],_,0).
count([X|T],X,Y):- !, count(T,X,Z), Y is 1+Z.
count([_|T],X,Z):- count(T,X,Z).

listCount(ElemToFind, NumberOccur, List) :-
            count(List, ElemToFind, NumberOccur).

updateList(NewList) :- list(L), append(L, NewList, Result),
                        print('New list is: '), print(Result), nl,
                        retract(list(L)), assert(list(Result)).

coordsCount(North, West, South, East, List) :-
                            listCount('N', North, List),
                            listCount('W', West, List),
                            listCount('S', South, List),
                            listCount('E', East, List).

indexOf([Element|_], Element, 0):- !.
indexOf([_|Tail], Element, Index):-
  indexOf(Tail, Element, Index1),
  !,
  Index is Index1+1.

minInList([Min],Min).                 % We have found the minimum

minInList([H,K|T],M) :-
    H =< K,                             % H is less than or equal to K
    minInList([H|T],M).               % so use H

minInList([H,K|T],M) :-
    H > K,                              % H is greater than K
    minInList([K|T],M).

% findMin ritorna la direzione meno scelta riferente la mappa (non al robot)
findMin(List, Return) :-
                    coordsCount(N, W, S, E, List),
                    L = [N, W, S, E],
                    print('List:'), print(L), nl,
                    minInList(L, Minimum),
                    indexOf(L, Minimum, Index),
                    (
                        Index =:= 0 -> Return = 'North'
                        ;
                        Index =:= 1 -> Return = 'West'
                        ;
                        Index =:= 2 -> Return = 'South'
                        ;
                        Index =:= 3 -> Return = 'East'
                    ).

% Determine least visited direction, update currentDegree and then calculate the coord wrt to robot system.
takeDecision(D):-  degree(CurrentDegree), list(L), findMin(L, R), 
                            print('CurrentDegree is: '), print(CurrentDegree), nl, 
                            print('Direction wrt to map is: '), print(R), nl,
                            (
                                R == 'North' ->
                                    (   CurrentDegree  =:= 0, hasValue('NorthSensor', 'False') ->
                                        updateList(['N']),
                                        D = 'North'
                                    ;   CurrentDegree =:= 90, hasValue('EastSensor', 'False') ->
                                        updateList(['N']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(0)),
                                        D = 'East'
                                    ;   CurrentDegree =:= 180, hasValue('SouthSensor', 'False') ->
                                        updateList(['N']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(0)),
                                        D = 'South'
                                    ;   CurrentDegree =:= 270, hasValue('WestSensor', 'False') ->
                                        updateList(['N']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(0)),
                                        D = 'West'
                                    ;   print('busy north'), nl, emptyDirection(D, CurrentDegree)
                                    )
                                ;
                                R == 'West' ->  
                                    (   CurrentDegree  =:= 0, hasValue('WestSensor', 'False') ->
                                        updateList(['W']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(90)),
                                        D = 'West'
                                    ;   CurrentDegree =:= 90, hasValue('NorthSensor', 'False') ->
                                        updateList(['W']),
                                        D = 'North'
                                    ;   CurrentDegree =:= 180, hasValue('EastSensor', 'False') ->
                                        updateList(['W']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(90)),
                                        D = 'East'
                                    ;   CurrentDegree =:= 270, hasValue('SouthSensor', 'False') -> 
                                        updateList(['W']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(90)),
                                        D = 'South'
                                    ; print('busy west'), nl, emptyDirection(D, CurrentDegree)
                                    )   
                                ;
                                R == 'East' ->
                                    (   CurrentDegree  =:= 0, hasValue('EastSensor', 'False') ->
                                        updateList(['E']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(270)),
                                        D = 'East'
                                    ;   CurrentDegree =:= 90, hasValue('SouthSensor', 'False') ->
                                        updateList(['E']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(270)),
                                        D = 'South'
                                    ;   CurrentDegree =:= 180, hasValue('WestSensor', 'False') ->
                                        updateList(['E']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(270)),
                                        D = 'West'
                                    ;   CurrentDegree =:= 270, hasValue('NorthSensor', 'False') -> 
                                        updateList(['E']),
                                        D = 'North'
                                    ;   print('busy east'), nl, emptyDirection(D, CurrentDegree)
                                    )
                                ;
                                R == 'South' ->
                                    (   CurrentDegree  =:= 0, hasValue('SouthSensor', 'False') ->
                                        updateList(['S']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(180)),
                                        D = 'South'
                                    ;   CurrentDegree =:= 90, hasValue('WestSensor', 'False') ->
                                        updateList(['S']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(180)),
                                        D = 'West'
                                    ;   CurrentDegree =:= 180, hasValue('NorthSensor', 'False') ->
                                        updateList(['S']),
                                        D = 'North'
                                    ;   CurrentDegree =:= 270, hasValue('EastSensor', 'False') ->
                                        updateList(['S']),
                                        retract(degree(CurrentDegree)), 
                                        assert(degree(180)),
                                        D = 'East'
                                    ;   print('busy south'), nl, emptyDirection(D, CurrentDegree)
                                    )
                            ).

% emptyDirection('North', CurrentDegree) :-
%     hasValue('NorthSensor', 'False'),print('eccolo1'), !.

% emptyDirection('West', CurrentDegree) :-
%     hasValue('WestSensor', 'False'),
%     NewDegree is mod(CurrentDegree+90, 360),
%     retract(degree(CurrentDegree)),
%     assert(degree(NewDegree)), print('eccolo2'),
%     !.

% emptyDirection('East', CurrentDegree) :-
%     hasValue('EastSensor', 'False'),
%     NewDegree is mod(CurrentDegree+180, 360),
%     retract(degree(CurrentDegree)),
%     assert(degree(NewDegree)), print('eccolo3'),
%     !.

% emptyDirection('South', CurrentDegree) :-
%     hasValue('SouthSensor', 'False'),
%     NewDegree is mod(CurrentDegree+270, 360),
%     retract(degree(CurrentDegree)),
%     assert(degree(NewDegree)),print('eccolo4'),
%     !.

% Determine empty direction, update currentDegree and then calculate the coord wrt to map system.
emptyDirection(D, CurrentDegree) :-
                                print('finding emptyDirection'),nl,
                                print('CurrentDegree is: '), print(CurrentDegree), nl, 
                                (
                                hasValue('NorthSensor', 'False') -> print('North is empty'),
                                    print('North is empty'), nl,
                                    D = 'North',
                                    (   CurrentDegree  =:= 0 ->
                                        updateList(['N'])
                                    ;   CurrentDegree =:= 90 ->
                                        updateList(['E'])
                                    ;   CurrentDegree =:= 180 ->
                                        updateList(['S'])
                                    ;   CurrentDegree =:= 270 ->
                                        updateList(['W'])
                                    )
                                ;
                               hasValue('WestSensor', 'False') -> print('West is empty'),
                                    print('West is empty'), nl,
                                    D = 'West',
                                    NewDegree is mod(CurrentDegree+90, 360),
                                    retract(degree(CurrentDegree)),
                                    assert(degree(NewDegree)),
                                    (   CurrentDegree  =:= 0 ->
                                        updateList(['W'])
                                    ;   CurrentDegree =:= 90 ->
                                        updateList(['N'])
                                    ;   CurrentDegree =:= 180 ->
                                        updateList(['E'])
                                    ;   CurrentDegree =:= 270 -> 
                                        updateList(['S'])
                                    )   
                                ;
                                hasValue('EastSensor', 'False') -> print('East is empty'),
                                    print('East is empty'), nl,
                                    D = 'East',
                                    NewDegree is mod(CurrentDegree+180, 360),
                                    retract(degree(CurrentDegree)),
                                    assert(degree(NewDegree)),
                                    (   CurrentDegree  =:= 0 ->
                                        updateList(['E'])
                                    ;   CurrentDegree =:= 90 ->
                                        updateList(['S'])
                                    ;   CurrentDegree =:= 180 ->
                                        updateList(['W'])
                                    ;   CurrentDegree =:= 270 -> 
                                        updateList(['N'])
                                    )
                                ;
                                hasValue('SouthSensor', 'False') -> 
                                    print('South is empty'), nl,
                                    D = 'South',
                                    NewDegree is mod(CurrentDegree+270, 360),
                                    retract(degree(CurrentDegree)),
                                    assert(degree(NewDegree)),
                                    (   CurrentDegree  =:= 0 ->
                                        updateList(['S'])
                                    ;   CurrentDegree =:= 90 ->
                                        updateList(['W'])
                                    ;   CurrentDegree =:= 180 ->
                                        updateList(['N'])
                                    ;   CurrentDegree =:= 270 ->
                                        updateList(['E'])
                                    )
                            ).


