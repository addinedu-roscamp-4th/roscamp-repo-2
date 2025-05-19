import React from 'react';
import { useUnifiedWebSockets } from '../context/UnifiedWebSocketProvider';
import { updateTableStatus } from '../api/orderApi';
import styled from 'styled-components';

const Overlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.85);
  z-index: 999;
  display: flex;
  justify-content: center;
  align-items: center;
`;

const CleaningButtonContainer = styled.div`
  position: relative;
  z-index: 1000;
  text-align: center;
`;

const CleaningButton = styled.button`
  padding: 80px 160px;
  font-size: 80px;
  background-color: #4CAF50;
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;

  &:hover {
    background-color: #45a049;
    transform: translateY(-2px);
    box-shadow: 0 6px 8px rgba(0, 0, 0, 0.15);
  }

  &:active {
    transform: translateY(0);
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  }
`;

const Message = styled.p`
  color: white;
  font-size: 18px;
  margin-top: 16px;
  text-align: center;
`;

const CleaningButtonComponent = () => {
  const { data } = useUnifiedWebSockets();
  const [isLoading, setIsLoading] = React.useState(false);

  // 현재 테이블 ID 가져오기
  const currentTableId = parseInt(localStorage.getItem('kioskTableId') || '1');

  // 현재 테이블의 상태 확인
  const currentTable = data.tables?.tables?.find(table => table['Table.id'] === currentTableId);
  const isCleaning = currentTable?.['Table.status'] === 'CLEANING';

  const handleCleaningComplete = async () => {
    if (isLoading) return;
    
    try {
      setIsLoading(true);
      await updateTableStatus(currentTableId, 'AVAILABLE');
    } catch (error) {
      console.error('청소 완료 처리 중 오류 발생:', error);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isCleaning) return null;

  return (
    <Overlay>
      <CleaningButtonContainer>
        <CleaningButton 
          onClick={handleCleaningComplete}
          disabled={isLoading}
        >
          {isLoading ? '처리 중...' : '청소 완료'}
        </CleaningButton>
        <Message>테이블 청소가 완료되면 버튼을 눌러주세요</Message>
      </CleaningButtonContainer>
    </Overlay>
  );
};

export default CleaningButtonComponent; 