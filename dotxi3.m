function dotxi3 = dotxi3(qdot,xi1,xi2,xi3)
 dotxi3 = qdot(1)*liebracket_426_new2(xi1,xi3) + qdot(2)*liebracket_426_new2(xi2,xi3);
 dotxi3=dotxi3';
end