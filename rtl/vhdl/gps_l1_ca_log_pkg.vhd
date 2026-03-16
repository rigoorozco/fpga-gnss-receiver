library ieee;
use ieee.std_logic_1164.all;
use std.textio.all;

package gps_l1_ca_log_pkg is
  type log_level_t is (LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR);

  procedure log_msg(msg : in string);
  procedure log_msg(tag : in string; msg : in string);
  procedure log_msg(level : in log_level_t; msg : in string);
  procedure log_msg(level : in log_level_t; tag : in string; msg : in string);
  procedure log_if(enabled : in boolean; msg : in string);
  procedure log_if(enabled : in boolean; level : in log_level_t; msg : in string);
end package;

package body gps_l1_ca_log_pkg is
  function level_prefix(level : log_level_t) return string is
  begin
    case level is
      when LOG_DEBUG => return "DEBUG";
      when LOG_INFO  => return "INFO";
      when LOG_WARN  => return "WARN";
      when LOG_ERROR => return "ERROR";
    end case;
  end function;

  procedure emit(msg : in string) is
    variable l : line;
  begin
    write(l, msg);
    writeline(output, l);
  end procedure;

  procedure log_msg(msg : in string) is
  begin
    emit(msg);
  end procedure;

  procedure log_msg(tag : in string; msg : in string) is
  begin
    emit("[" & tag & "] " & msg);
  end procedure;

  procedure log_msg(level : in log_level_t; msg : in string) is
  begin
    emit("[" & level_prefix(level) & "] " & msg);
  end procedure;

  procedure log_msg(level : in log_level_t; tag : in string; msg : in string) is
  begin
    emit("[" & level_prefix(level) & "][" & tag & "] " & msg);
  end procedure;

  procedure log_if(enabled : in boolean; msg : in string) is
  begin
    if enabled then
      emit(msg);
    end if;
  end procedure;

  procedure log_if(enabled : in boolean; level : in log_level_t; msg : in string) is
  begin
    if enabled then
      emit("[" & level_prefix(level) & "] " & msg);
    end if;
  end procedure;
end package body;
