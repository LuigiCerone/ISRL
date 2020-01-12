:- dynamic(list/1).

% Per come è fatto adesso asserziona le percezioni passate e con takeDecision sceglie dove andare.
% perception(['False', 'False', 'False', 'False'])
% False= Non ho ostacolo;
% True = Ho un ostacolo;
hasValue('NorthSensor', S1) :- perception([S1, _, _, _]).
hasValue('WestSensor', S2) :- perception([_, S2, _, _]).
hasValue('EastSensor', S3) :- perception([_, _, S3, _]).
hasValue('SouthSensor', S4) :- perception([_, _, _, S4]).


degree(0). % Per contenere orientamento.

% list([]). % Per le direzioni precedentemente prese.
list(['W', 'W', 'S']).

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

findMin(List) :-
                    coordsCount(N, W, S, E, List),
                    L = [N, W, S, E],
                    print('List:'), print(L), nl,
                    minInList(List, Minimum),
                    indexOf(List, Minimum, Index),
                    print('The index of the minimum:'), print(Index), assert(minimum(Index)), nl.

computeDirectionEnv('North'):-  degree(CurrentDegree),
                            (   CurrentDegree  =:= 0, hasValue('NorthSensor', 'False') ->
                                updateList(['N']), takeDecision('North')
                            ;   CurrentDegree =:= 90, hasValue('EastSensor', 'False') ->
                                updateList(['N']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(0)),
                                takeDecision('East')
                            ;   CurrentDegree =:= 180, hasValue('SouthSensor', 'False') ->
                                updateList(['N']), 
                                retract(degree(CurrentDegree)), 
                                assert(degree(0)),
                                takeDecision(['South'])
                            ;   CurrentDegree =:= 270, hasValue('WestSensor', 'False') -> 
                                updateList(['N']), 
                                retract(degree(CurrentDegree)), 
                                assert(degree(0)),
                                takeDecision(['West'])
                            ).

computeDirectionEnv('West'):-  degree(CurrentDegree),
                            (   CurrentDegree  =:= 0, hasValue('WestSensor', 'False') ->
                                updateList(['W']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(90)),
                                takeDecision(['West'])
                            ;   CurrentDegree =:= 90, hasValue('NorthSensor', 'False') ->
                                updateList(['W']),
                                takeDecision(['North'])
                            ;   CurrentDegree =:= 180, hasValue('EastSensor', 'False') ->
                                updateList(['W']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(90)),
                                takeDecision(['East'])
                            ;   CurrentDegree =:= 270, hasValue('SouthSensor', 'False') -> 
                                updateList(['W']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(90)),
                                takeDecision(['South'])
                            ).

computeDirectionEnv('East'):-  degree(CurrentDegree),
                            (   CurrentDegree  =:= 0, hasValue('EastSensor', 'False') ->
                                updateList(['E']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(270)),
                                takeDecision(['East'])
                            ;   CurrentDegree =:= 90, hasValue('SouthSensor', 'False') ->
                                updateList(['E']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(270)),
                                takeDecision(['South'])
                            ;   CurrentDegree =:= 180, hasValue('WestSensor', 'False') ->
                                updateList(['E']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(270)),
                                takeDecision(['West'])
                            ;   CurrentDegree =:= 270, hasValue('NorthSensor', 'False') -> 
                                updateList(['E']),
                                takeDecision(['North'])
                            ).

computeDirectionEnv('South'):-  degree(CurrentDegree),
                            (   CurrentDegree  =:= 0, hasValue('SouthSensor', 'False') ->
                                updateList(['S']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(180)),
                                takeDecision(['South'])
                            ;   CurrentDegree =:= 90, hasValue('WestSensor', 'False') ->
                                updateList(['S']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(180)),
                                takeDecision(['West'])
                            ;   CurrentDegree =:= 180, hasValue('NorthSensor', 'False') ->
                                updateList(['S']),
                                takeDecision(['North'])
                            ;   CurrentDegree =:= 270, hasValue('EastSensor', 'False') -> 
                                updateList(['S']),
                                retract(degree(CurrentDegree)), 
                                assert(degree(180)),
                                takeDecision(['East'])
                            ).

% takeDecision('North') :-
%    hasValue('NorthSensor', 'False'), !.

% takeDecision('West') :-
%    hasValue('WestSensor', 'False'), !.

% takeDecision('East') :-
%    hasValue('EastSensor', 'False'), !.

% takeDecision('South') :-
%    hasValue('SouthSensor', 'False').


