

struct Question {
  String question;
  String correctAwnser;
  //if empty you need to write a number like 1986 if not you have to choose between A, B, C, D ig
  String awnsers[4];
};


const Question questions[2] = {
  { "You Gay?", "YES", { "YES", "NO" } },
  { "You not Gay?", "NO", { "YES", "NO" } }
};