:- dynamic(list/1).
:- dynamic(degree/1).

% Per come è fatto adesso asserziona le percezioni passate e con takeDecision sceglie dove andare.
% perception(['False', 'False', 'False', 'False'])
% False= Non ho ostacolo;
% True = Ho un ostacolo;
perception(['False', 'False', 'False', 'False']).
hasValue('NorthSensor', S1) :- perception([S1, _, _, _]).
hasValue('WestSensor', S2) :- perception([_, S2, _, _]).
hasValue('EastSensor', S3) :- perception([_, _, S3, _]).
hasValue('SouthSensor', S4) :- perception([_, _, _, S4]).


degree(0). % Per contenere orientamento.

% list([]). % Per le direzioni precedentemente prese.
list(['W', 'W', 'S', 'N']).

count([],_,0).
count([X|T],X,Y):- !, count(T,X,Z), Y is 1+Z.
count([_|T],X,Z):- count(T,X,Z).

listCount(ElemToFind, NumberOccur, List) :-
            count(List, ElemToFind, NumberOccur).

updateList(NewList) :- list(L), append(L, NewList, Result),
                        print(Result),
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

takeDecision(D):-  degree(CurrentDegree), list(L), findMin(L, R), 
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
                                    )
                            ).

% takeDecision('North') :-
%    hasValue('NorthSensor', 'False'), !.

% takeDecision('West') :-
%    hasValue('WestSensor', 'False'), !.

% takeDecision('East') :-
%    hasValue('EastSensor', 'False'), !.

% takeDecision('South') :-
%    hasValue('SouthSensor', 'False').


